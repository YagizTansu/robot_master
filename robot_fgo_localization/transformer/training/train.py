"""
Training script for TrustTransformer.

Loss
----
Each batch:
    L_error  = MSE(pred_pos_err, pos_err_m)
    gt_weight = sigmoid(pos_err_m / err_scale)      -- GT-supervised scaling
    L_trust  = mean(gt_weight * MSE(pred_trust, pseudo_trust))
    L_yaw    = MSE(pred_pos_err, yaw_err_rad)        -- auxiliary, small λ

    total = λ_error * L_error + λ_trust * L_trust + λ_yaw * L_yaw

Usage
-----
    python -m training.train                         # default config
    python -m training.train --data_dir ~/mydata --epochs 80
"""

import argparse
import math
import os
import random
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from .config import ModelConfig, TrainConfig
from .dataset import build_datasets
from .model import TrustTransformer


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _gt_weight(pos_err: torch.Tensor, scale: float) -> torch.Tensor:
    """
    sigmoid(pos_err / scale) — upweights trust loss when GT error is large.
    Returns a per-sample weight tensor with the same shape as pos_err.
    """
    return torch.sigmoid(pos_err / scale)


def _mse_per_sample(pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """MSE without reduction (result has same batch dimension)."""
    return (pred - target).pow(2).mean(dim=-1)   # mean over output dims


# ─────────────────────────────────────────────────────────────────────────────
# Training + validation steps
# ─────────────────────────────────────────────────────────────────────────────

def _run_epoch(
    model:      TrustTransformer,
    loader:     DataLoader,
    cfg:        TrainConfig,
    optimizer:  torch.optim.Optimizer | None,
    device:     torch.device,
    is_train:   bool,
) -> dict:
    """
    Run one full epoch.  Returns a dict of average loss components.
    Pass optimizer=None for the validation pass.
    """
    model.train(is_train)
    total_loss = total_l_err = total_l_trust = total_l_yaw = 0.0
    n_batches = 0

    ctx = torch.enable_grad() if is_train else torch.no_grad()
    with ctx:
        for feat, pos_err, yaw_err, pseudo_trust in loader:
            feat         = feat.to(device)              # [B, seq, n_feat]
            pos_err      = pos_err.to(device)           # [B]
            yaw_err      = yaw_err.to(device)           # [B]
            pseudo_trust = pseudo_trust.to(device)      # [B, 4]

            trust_pred, err_pred, yaw_pred = model(feat)  # [B,4], [B,1], [B,1]
            err_pred_sq = err_pred.squeeze(1)              # [B]
            yaw_pred_sq = yaw_pred.squeeze(1)              # [B]

            # ── Error prediction loss ────────────────────────────────────────
            l_error = nn.functional.mse_loss(err_pred_sq, pos_err)

            # ── Trust pseudo-label loss (GT-weighted per sample) ─────────────
            w = _gt_weight(pos_err, cfg.err_scale)           # [B]
            per_sample_trust_mse = _mse_per_sample(
                trust_pred, pseudo_trust)                     # [B]
            l_trust = (w * per_sample_trust_mse).mean()

            # ── Auxiliary yaw loss (yaw_head vs yaw_err_rad, both in radians) ────
            # Previously pred_pos_err (metres) was compared to yaw_err_rad (radians)
            # causing a dimensional mismatch. Now uses the dedicated yaw_head output.
            l_yaw = nn.functional.mse_loss(yaw_pred_sq, yaw_err)

            loss = (cfg.lambda_error * l_error
                    + cfg.lambda_trust * l_trust
                    + cfg.lambda_yaw   * l_yaw)

            if is_train:
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                optimizer.step()

            total_loss    += loss.item()
            total_l_err   += l_error.item()
            total_l_trust += l_trust.item()
            total_l_yaw   += l_yaw.item()
            n_batches     += 1

    n = max(n_batches, 1)
    return {
        "loss":    total_loss    / n,
        "l_error": total_l_err   / n,
        "l_trust": total_l_trust / n,
        "l_yaw":   total_l_yaw   / n,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Main training loop
# ─────────────────────────────────────────────────────────────────────────────

def train(cfg: TrainConfig, model_cfg: ModelConfig) -> None:
    _set_seed(cfg.seed)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[Train] Device: {device}")

    # ── Data ────────────────────────────────────────────────────────────────
    train_ds, val_ds, norm_stats = build_datasets(cfg)

    train_loader = DataLoader(
        train_ds,
        batch_size=cfg.batch_size,
        shuffle=True,
        num_workers=cfg.num_workers,
        pin_memory=device.type == "cuda",
        drop_last=True,       # avoids BatchNorm issues with size-1 batches
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=cfg.batch_size * 2,
        shuffle=False,
        num_workers=cfg.num_workers,
        pin_memory=device.type == "cuda",
    )

    if len(train_ds) == 0:
        raise RuntimeError(
            "Training dataset is empty. "
            "Collect more data with the feature_extractor_node first.")

    # ── Model ────────────────────────────────────────────────────────────────
    model = TrustTransformer(model_cfg).to(device)
    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"[Train] TrustTransformer — {n_params:,} trainable parameters.")

    # ── Optimizer + scheduler ────────────────────────────────────────────────
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)

    if cfg.lr_scheduler == "cosine":
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
            optimizer, T_max=cfg.epochs, eta_min=cfg.lr * 0.01)
    else:
        scheduler = torch.optim.lr_scheduler.StepLR(
            optimizer, step_size=cfg.lr_step_size, gamma=cfg.lr_gamma)

    # ── Checkpoint directory ─────────────────────────────────────────────────
    ckpt_dir = Path(os.path.expanduser(cfg.checkpoint_dir))
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    # ── Training loop ────────────────────────────────────────────────────────
    best_val_loss = math.inf
    best_ckpt     = ckpt_dir / "best.pt"

    print(f"\n{'='*60}")
    print(f"  Training for {cfg.epochs} epochs  |  batch={cfg.batch_size}")
    print(f"  λ_error={cfg.lambda_error}  λ_trust={cfg.lambda_trust}  "
          f"λ_yaw={cfg.lambda_yaw}  err_scale={cfg.err_scale}")
    print(f"{'='*60}\n")

    for epoch in range(1, cfg.epochs + 1):
        t0 = time.time()

        train_metrics = _run_epoch(
            model, train_loader, cfg, optimizer, device, is_train=True)
        val_metrics = _run_epoch(
            model, val_loader, cfg, optimizer=None, device=device, is_train=False)

        scheduler.step()

        elapsed = time.time() - t0
        print(
            f"Epoch {epoch:03d}/{cfg.epochs}  "
            f"train_loss={train_metrics['loss']:.5f}  "
            f"(err={train_metrics['l_error']:.5f}  "
            f"trust={train_metrics['l_trust']:.5f}  "
            f"yaw={train_metrics['l_yaw']:.5f})  |  "
            f"val_loss={val_metrics['loss']:.5f}  "
            f"lr={scheduler.get_last_lr()[0]:.2e}  "
            f"[{elapsed:.1f}s]"
        )

        # Save best checkpoint
        if val_metrics["loss"] < best_val_loss:
            best_val_loss = val_metrics["loss"]
            torch.save({
                "epoch":       epoch,
                "model_state": model.state_dict(),
                "model_cfg":   model_cfg,
                "val_loss":    best_val_loss,
                "norm_stats":  norm_stats,
            }, best_ckpt)
            print(f"  ✓ Best checkpoint saved → {best_ckpt}  "
                  f"(val_loss={best_val_loss:.5f})")

    # Save final checkpoint
    final_ckpt = ckpt_dir / "final.pt"
    torch.save({
        "epoch":       cfg.epochs,
        "model_state": model.state_dict(),
        "model_cfg":   model_cfg,
        "val_loss":    val_metrics["loss"],
        "norm_stats":  norm_stats,
    }, final_ckpt)
    print(f"\n[Train] Final checkpoint saved → {final_ckpt}")
    print(f"[Train] Best val_loss: {best_val_loss:.5f}  → {best_ckpt}")


# ─────────────────────────────────────────────────────────────────────────────
# CLI entry point
# ─────────────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Train TrustTransformer")
    p.add_argument("--data_dir",       default=None)
    p.add_argument("--checkpoint_dir", default=None)
    p.add_argument("--epochs",         type=int,   default=None)
    p.add_argument("--batch_size",     type=int,   default=None)
    p.add_argument("--lr",             type=float, default=None)
    p.add_argument("--seq_len",        type=int,   default=None)
    p.add_argument("--lambda_trust",   type=float, default=None)
    p.add_argument("--err_scale",      type=float, default=None)
    p.add_argument("--seed",           type=int,   default=None)
    return p.parse_args()


def main() -> None:
    args    = _parse_args()
    cfg     = TrainConfig()
    m_cfg   = ModelConfig()

    # Override defaults with CLI args (skip None values)
    for key, val in vars(args).items():
        if val is not None and hasattr(cfg, key):
            setattr(cfg, key, val)
    if args.seq_len is not None:
        cfg.seq_len  = args.seq_len
        m_cfg.seq_len = args.seq_len

    train(cfg, m_cfg)


if __name__ == "__main__":
    main()
