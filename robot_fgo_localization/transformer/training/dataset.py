"""
Dataset utilities for the FGO Transformer training pipeline.

Responsibilities
----------------
1.  Load one or more feature CSV files produced by feature_extractor_node.
2.  Drop rows with NaN in label columns; fill NaN in feature columns with a
    configurable constant (default 0.0 — "sensor not reporting" case).
3.  Compute per-feature mean/std normalization stats from the training split
    and apply them.  Stats are saved to a JSON file so the inference node can
    apply the same normalization without re-loading any CSV.
4.  Build a sliding-window dataset: each sample is a (seq_len × n_features)
    tensor + scalar labels + pseudo trust-weight labels.
5.  Compute rule-based pseudo trust-weight labels from raw feature values.
"""

import glob
import json
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset

from .config import (
    FEATURE_COLS,
    LABEL_ERR_COL,
    LABEL_YAW_COL,
    PSEUDO_FITNESS_THRESHOLD,
    PSEUDO_ACCEL_THRESHOLD,
    PSEUDO_JERK_THRESHOLD,
    PSEUDO_GPS_WEIGHT,
    TRUST_EPSILON,
    TrainConfig,
)


# ─────────────────────────────────────────────────────────────────────────────
# Low-level helpers
# ─────────────────────────────────────────────────────────────────────────────

def _pseudo_trust(df: pd.DataFrame) -> np.ndarray:
    """
    Compute rule-based pseudo trust-weight labels from raw feature columns.

    Returns an (N, 4) float32 array with columns [w_encoder, w_imu, w_lidar, w_gps].
    Each weight is clipped to [EPSILON, 1.0].

    Rules
    -----
    w_lidar   = 1 - clip(fitness_score   / FITNESS_THR,  0, 1)
    w_imu     = 1 - clip(accel_norm_dev  / ACCEL_THR,    0, 1)
    w_encoder = 1 - clip(jerk            / JERK_THR,     0, 1)
    w_gps     = PSEUDO_GPS_WEIGHT  (GPS disabled)

    NaN in input features → treated as 0 (sensor anomaly unknown → full trust).
    """
    def _safe(series: pd.Series) -> np.ndarray:
        return np.nan_to_num(series.to_numpy(dtype=np.float32), nan=0.0)

    fit   = _safe(df["fitness_score"])
    accel = _safe(df["accel_norm_dev"])
    jerk  = _safe(df["jerk"])

    w_lidar   = 1.0 - np.clip(fit   / PSEUDO_FITNESS_THRESHOLD, 0.0, 1.0)
    w_imu     = 1.0 - np.clip(accel / PSEUDO_ACCEL_THRESHOLD,   0.0, 1.0)
    w_encoder = 1.0 - np.clip(jerk  / PSEUDO_JERK_THRESHOLD,    0.0, 1.0)
    w_gps     = np.full(len(df), PSEUDO_GPS_WEIGHT, dtype=np.float32)

    pseudo = np.stack([w_encoder, w_imu, w_lidar, w_gps], axis=1)
    pseudo = np.clip(pseudo, TRUST_EPSILON, 1.0).astype(np.float32)
    return pseudo


def load_csv_files(data_dir: str) -> pd.DataFrame:
    """
    Load all features_*.csv files from data_dir into a single DataFrame.

    Each file is sorted by timestamp_sec before concatenation so that the
    sliding window always moves forward in time within each session.
    Files from different sessions are kept in order (sorted by filename,
    which encodes the recording timestamp).
    """
    pattern = os.path.join(os.path.expanduser(data_dir), "features_*.csv")
    paths   = sorted(glob.glob(pattern))

    if not paths:
        raise FileNotFoundError(
            f"No 'features_*.csv' files found in: {data_dir}\n"
            f"Run the feature_extractor_node first to collect training data."
        )

    frames: List[pd.DataFrame] = []
    for p in paths:
        df = pd.read_csv(p)
        df = df.sort_values("timestamp_sec").reset_index(drop=True)
        frames.append(df)

    print(f"[Dataset] Loaded {len(paths)} CSV file(s) — "
          f"{sum(len(f) for f in frames)} total rows.")
    return pd.concat(frames, ignore_index=True)


def compute_norm_stats(
    df: pd.DataFrame,
    feature_cols: List[str],
    nan_fill: float,
) -> Dict[str, Dict[str, float]]:
    """
    Compute per-feature mean and std from df (training split only).

    NaN values are filled with nan_fill BEFORE computing stats so the
    statistics reflect how the inference node will treat missing data.

    Returns a dict  {col_name: {"mean": float, "std": float}}.
    std=1.0 is used for constant columns to avoid division-by-zero.
    """
    stats: Dict[str, Dict[str, float]] = {}
    for col in feature_cols:
        vals = df[col].fillna(nan_fill).to_numpy(dtype=np.float64)
        mean = float(np.mean(vals))
        std  = float(np.std(vals))
        if std < 1e-8:
            std = 1.0   # constant column — preserve value without NaN explosion
        stats[col] = {"mean": mean, "std": std}
    return stats


def save_norm_stats(stats: Dict, path: str) -> None:
    os.makedirs(os.path.dirname(os.path.expanduser(path)), exist_ok=True)
    with open(os.path.expanduser(path), "w") as f:
        json.dump(stats, f, indent=2)
    print(f"[Dataset] Normalization stats saved → {path}")


def load_norm_stats(path: str) -> Dict:
    with open(os.path.expanduser(path)) as f:
        return json.load(f)


def apply_norm(
    arr: np.ndarray,
    stats: Dict[str, Dict[str, float]],
    feature_cols: List[str],
) -> np.ndarray:
    """Apply (x - mean) / std normalization column-wise."""
    arr = arr.copy()
    for i, col in enumerate(feature_cols):
        arr[:, i] = (arr[:, i] - stats[col]["mean"]) / stats[col]["std"]
    return arr


# ─────────────────────────────────────────────────────────────────────────────
# PyTorch Dataset
# ─────────────────────────────────────────────────────────────────────────────

class TrustWeightDataset(Dataset):
    """
    Sliding-window dataset over feature CSV rows.

    Each sample:
        features   : float32 tensor  [seq_len, n_features]  (normalized)
        pos_err    : float32 scalar   (pos_err_m label)
        yaw_err    : float32 scalar   (yaw_err_rad label)
        pseudo_trust: float32 tensor [4]  (rule-based trust pseudo-labels)

    Windows that cross a session boundary (detected by a timestamp gap larger
    than max_gap_sec) are silently discarded to avoid feeding stale-to-new
    transitions into the model.
    """

    def __init__(
        self,
        features:     np.ndarray,        # (N, n_features) — normalized
        pos_errors:   np.ndarray,        # (N,) pos_err_m
        yaw_errors:   np.ndarray,        # (N,) yaw_err_rad
        pseudo_trust: np.ndarray,        # (N, 4) rule-based trust weights
        timestamps:   np.ndarray,        # (N,) timestamp_sec — for gap detection
        seq_len:      int,
        step:         int   = 1,
        max_gap_sec:  float = 5.0,       # timestamps farther than this → new session
    ) -> None:
        super().__init__()
        self._features     = features.astype(np.float32)
        self._pos_errors   = pos_errors.astype(np.float32)
        self._yaw_errors   = yaw_errors.astype(np.float32)
        self._pseudo_trust = pseudo_trust.astype(np.float32)

        # Build index of valid window end positions (skip session boundaries)
        self._valid_ends: List[int] = []
        for end in range(seq_len - 1, len(features), step):
            start = end - seq_len + 1
            # Check for timestamp gaps within the window
            gap = np.max(np.diff(timestamps[start : end + 1]))
            if gap < max_gap_sec:
                self._valid_ends.append(end)

        print(f"[Dataset] {len(self._valid_ends)} valid windows "
              f"(seq_len={seq_len}, step={step}, "
              f"excluded {(len(features) - seq_len + 1) - len(self._valid_ends)} "
              f"cross-session windows).")

    def __len__(self) -> int:
        return len(self._valid_ends)

    def __getitem__(self, idx: int):
        end   = self._valid_ends[idx]
        start = end - len(self._features[0]) + 1   # not needed; use seq_len field
        # Recover seq_len from the window size used at construction
        seq_len = self._valid_ends[0] - (self._valid_ends[0] - len(self._features[0]) + 1) + 1 \
                  if self._valid_ends else 1
        # Simpler: infer seq_len from first valid window
        first_end   = self._valid_ends[0]
        # We store seq_len during construction — but to avoid attribute,
        # infer as end - start + 1 using the first window gap-free.
        # Actually simplest: always use fixed seq_len from module constant.
        # We save it as an instance attribute below (set in factory).
        start = end - self._seq_len + 1

        feat   = torch.from_numpy(self._features[start : end + 1])      # [seq, feat]
        p_err  = torch.tensor(self._pos_errors[end],   dtype=torch.float32)
        y_err  = torch.tensor(self._yaw_errors[end],   dtype=torch.float32)
        pseudo = torch.from_numpy(self._pseudo_trust[end])               # [4]

        return feat, p_err, y_err, pseudo

    # seq_len is injected by the factory function below
    _seq_len: int = 20


# ─────────────────────────────────────────────────────────────────────────────
# Factory — build train / val splits from CSV directory
# ─────────────────────────────────────────────────────────────────────────────

def build_datasets(
    cfg: TrainConfig,
    norm_stats: Optional[Dict] = None,
) -> Tuple["TrustWeightDataset", "TrustWeightDataset", Dict]:
    """
    Load CSVs, compute or apply normalization, compute pseudo-labels, and
    return (train_dataset, val_dataset, norm_stats).

    If norm_stats is None, stats are computed from the training split and
    saved to cfg.stats_path.  Pass saved stats at inference time.
    """
    df = load_csv_files(cfg.data_dir)

    # ── Drop rows with missing label ────────────────────────────────────────
    before = len(df)
    df = df.dropna(subset=[LABEL_ERR_COL, LABEL_YAW_COL]).reset_index(drop=True)
    print(f"[Dataset] Dropped {before - len(df)} rows with NaN labels. "
          f"{len(df)} rows remain.")

    # ── Fill NaN in features ────────────────────────────────────────────────
    feature_df = df[FEATURE_COLS].fillna(cfg.nan_fill)

    # ── Train / val split (time-ordered → split at fixed index, no shuffle) ─
    split = int(len(df) * (1.0 - cfg.val_fraction))
    train_df   = feature_df.iloc[:split]
    val_df     = feature_df.iloc[split:]
    timestamps = df["timestamp_sec"].to_numpy(dtype=np.float64)

    # ── Normalization stats ──────────────────────────────────────────────────
    if norm_stats is None:
        norm_stats = compute_norm_stats(train_df, FEATURE_COLS, cfg.nan_fill)
        save_norm_stats(norm_stats, cfg.stats_path)

    train_arr = apply_norm(train_df.to_numpy(dtype=np.float32), norm_stats, FEATURE_COLS)
    val_arr   = apply_norm(val_df.to_numpy(dtype=np.float32),   norm_stats, FEATURE_COLS)

    # ── Labels ──────────────────────────────────────────────────────────────
    pos_errors = df[LABEL_ERR_COL].to_numpy(dtype=np.float32)
    yaw_errors = df[LABEL_YAW_COL].to_numpy(dtype=np.float32)

    # ── Pseudo trust labels ──────────────────────────────────────────────────
    # Computed from RAW df (before normalization) so thresholds stay meaningful.
    pseudo_all = _pseudo_trust(df)

    def _make(arr, idx_start, idx_end):
        ds = TrustWeightDataset(
            features     = arr,
            pos_errors   = pos_errors[idx_start:idx_end],
            yaw_errors   = yaw_errors[idx_start:idx_end],
            pseudo_trust = pseudo_all[idx_start:idx_end],
            timestamps   = timestamps[idx_start:idx_end],
            seq_len      = cfg.seq_len,
            step         = cfg.step,
        )
        ds._seq_len = cfg.seq_len
        return ds

    train_ds = _make(train_arr, 0,     split)
    val_ds   = _make(val_arr,   split, len(df))

    print(f"[Dataset] Train: {len(train_ds)} windows  |  "
          f"Val: {len(val_ds)} windows")
    return train_ds, val_ds, norm_stats
