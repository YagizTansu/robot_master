"""
TrustTransformer — dual-head Transformer model.

Architecture
------------
Input  : [batch, seq_len, n_features]
         Normalized feature window (11 features × 20 timesteps)

Pipeline:
  1.  Linear input projection   n_features → d_model
  2.  Sinusoidal positional encoding (added, not concatenated)
  3.  TransformerEncoder         n_layers × (self-attention + FFN)
  4.  Mean pooling over sequence dimension   →  [batch, d_model]
  5a. error_head:  Linear → Softplus   →  pred_pos_err  [batch, 1]  ≥ 0
  5b. trust_head:  Linear → Sigmoid    →  trust_weights [batch, 4]  ∈ (0, 1]

Trust weight index mapping  (matches TrustWeights C++ struct):
  0 = w_encoder   1 = w_imu   2 = w_lidar   3 = w_gps
"""

import math

import torch
import torch.nn as nn
from torch import Tensor

from .config import ModelConfig, TRUST_EPSILON


# ─────────────────────────────────────────────────────────────────────────────
# Positional Encoding
# ─────────────────────────────────────────────────────────────────────────────

class SinusoidalPositionalEncoding(nn.Module):
    """
    Standard fixed sinusoidal positional encoding (Vaswani et al., 2017).

    pe[pos, 2i]   = sin(pos / 10000^(2i / d_model))
    pe[pos, 2i+1] = cos(pos / 10000^(2i / d_model))

    Registered as a buffer (not a parameter) so it moves to GPU with .to(device)
    but is never updated by the optimizer.
    """

    def __init__(self, d_model: int, max_len: int = 512, dropout: float = 0.1) -> None:
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)

        pe = torch.zeros(max_len, d_model)                         # [max_len, d_model]
        pos = torch.arange(max_len, dtype=torch.float).unsqueeze(1)  # [max_len, 1]
        div = torch.exp(
            torch.arange(0, d_model, 2, dtype=torch.float)
            * (-math.log(10000.0) / d_model)
        )                                                           # [d_model/2]
        pe[:, 0::2] = torch.sin(pos * div)
        pe[:, 1::2] = torch.cos(pos * div)
        self.register_buffer("pe", pe.unsqueeze(0))                # [1, max_len, d_model]

    def forward(self, x: Tensor) -> Tensor:
        """x: [batch, seq, d_model]"""
        x = x + self.pe[:, : x.size(1), :]
        return self.dropout(x)


# ─────────────────────────────────────────────────────────────────────────────
# Main model
# ─────────────────────────────────────────────────────────────────────────────

class TrustTransformer(nn.Module):
    """
    Dual-head Transformer for sensor trust-weight prediction.

    Forward returns (trust_weights, pred_pos_err):
        trust_weights : [batch, 4]  ∈ (EPSILON, 1.0]
        pred_pos_err  : [batch, 1]  ≥ 0  (metres)
    """

    def __init__(self, cfg: ModelConfig) -> None:
        super().__init__()
        self.cfg = cfg

        # ── Input projection ─────────────────────────────────────────────────
        self.input_proj = nn.Linear(cfg.n_features, cfg.d_model)

        # ── Positional encoding ──────────────────────────────────────────────
        self.pos_enc = SinusoidalPositionalEncoding(
            cfg.d_model, max_len=cfg.seq_len + 4, dropout=cfg.dropout)

        # ── Transformer encoder ──────────────────────────────────────────────
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=cfg.d_model,
            nhead=cfg.n_heads,
            dim_feedforward=cfg.d_model * 4,   # standard 4× FFN expansion
            dropout=cfg.dropout,
            batch_first=True,                  # [batch, seq, d_model]
            norm_first=True,                   # Pre-LN: more stable for small models
        )
        self.transformer = nn.TransformerEncoder(
            encoder_layer, num_layers=cfg.n_layers)

        # ── Output heads ─────────────────────────────────────────────────────

        # error_head: predict pos_err_m (non-negative regression)
        self.error_head = nn.Sequential(
            nn.Linear(cfg.d_model, cfg.d_model // 2),
            nn.GELU(),
            nn.Linear(cfg.d_model // 2, 1),
            nn.Softplus(),          # guarantees pred_err ≥ 0
        )

        # trust_head: predict trust weights ∈ (0, 1]
        # Sigmoid output is scaled to [EPSILON, 1.0] by the forward pass.
        self.trust_head = nn.Sequential(
            nn.Linear(cfg.d_model, cfg.d_model // 2),
            nn.GELU(),
            nn.Linear(cfg.d_model // 2, 4),
            nn.Sigmoid(),           # raw output ∈ (0, 1)
        )

        self._init_weights()

    def _init_weights(self) -> None:
        """Xavier uniform for linear layers; zero bias initialisation."""
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.xavier_uniform_(module.weight)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)

    def forward(self, x: Tensor) -> tuple[Tensor, Tensor]:
        """
        Parameters
        ----------
        x : [batch, seq_len, n_features]  — normalized feature window

        Returns
        -------
        trust_weights : [batch, 4]  — clipped to [EPSILON, 1.0]
        pred_pos_err  : [batch, 1]  — non-negative error prediction (metres)
        """
        # Project features to d_model
        h = self.input_proj(x)                    # [B, seq, d_model]
        h = self.pos_enc(h)                       # [B, seq, d_model]
        h = self.transformer(h)                   # [B, seq, d_model]

        # Mean pooling across sequence dimension
        h_pool = h.mean(dim=1)                    # [B, d_model]

        # Dual heads
        pred_err    = self.error_head(h_pool)     # [B, 1]
        trust_raw   = self.trust_head(h_pool)     # [B, 4]  ∈ (0, 1)

        # Scale sigmoid output to [EPSILON, 1.0]
        # sigmoid(x) ∈ (0,1)  →  EPSILON + sigmoid(x) * (1 - EPSILON) ∈ (EPSILON, 1.0)
        trust_weights = TRUST_EPSILON + trust_raw * (1.0 - TRUST_EPSILON)

        return trust_weights, pred_err

    # ── Convenience for inference (single-window, no batch dimension) ────────

    @torch.no_grad()
    def predict(self, window: Tensor) -> tuple[Tensor, Tensor]:
        """
        Parameters
        ----------
        window : [seq_len, n_features]  — single normalized window (no batch dim)

        Returns
        -------
        trust_weights : [4]   — detached CPU tensor
        pred_pos_err  : scalar float
        """
        self.eval()
        trust, err = self(window.unsqueeze(0))    # add batch dim
        return trust.squeeze(0).cpu(), err.item()
