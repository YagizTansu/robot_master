"""
Training hyperparameters and schema constants.

All magic numbers live here — nowhere else in the training code.
"""

from dataclasses import dataclass, field
from typing import List


# ─────────────────────────────────────────────────────────────────────────────
# CSV schema
# ─────────────────────────────────────────────────────────────────────────────

# The 9 columns fed as input features to the Transformer.
# Order here defines column order in the normalisation stats file.
FEATURE_COLS: List[str] = [
    "fitness_score",    # NDT match quality        (lower = better)
    "fitness_age_sec",  # staleness of last scan   (higher = scanner idle)
    "accel_norm_dev",   # |‖accel‖ - g|            (vibration / shock proxy)
    "gyro_z",           # angular rate z            (rotation)
    "accel_norm_std",   # std-dev of ‖accel‖        (vibration envelope)
    "gyro_z_std",       # std-dev of gyro_z         (rotation flutter)
    "linear_vel",       # wheel-odometry vx         (m/s)
    "angular_vel",      # wheel-odometry wz         (rad/s)
    "jerk",             # |Δvx / Δt|               (slip proxy)
]

# Primary training target column
LABEL_ERR_COL:  str = "pos_err_m"

# Supplementary label (used as a second loss term with small weight)
LABEL_YAW_COL:  str = "yaw_err_rad"

# Output index mapping – [encoder, imu, lidar, gps]
TRUST_IDX_ENCODER: int = 0
TRUST_IDX_IMU:     int = 1
TRUST_IDX_LIDAR:   int = 2
TRUST_IDX_GPS:     int = 3


# ─────────────────────────────────────────────────────────────────────────────
# Rule-based pseudo-label thresholds
# ─────────────────────────────────────────────────────────────────────────────
# These turn raw feature values into [0, 1] trust pseudo-labels.
# w  = clip(1 - raw_feature / threshold, EPSILON, 1.0)
# At raw_feature == 0  → w = 1.0  (full trust)
# At raw_feature >= threshold → w ≈ EPSILON  (near-zero trust)

PSEUDO_FITNESS_THRESHOLD:   float = 0.15   # NDT fitness score;  see fgo_params fitness_score_threshold
PSEUDO_ACCEL_THRESHOLD:     float = 2.0    # m/s²  — clear shock / excessive vibration
PSEUDO_JERK_THRESHOLD:      float = 5.0    # m/s²  — wheel slip / sudden stop
PSEUDO_GPS_WEIGHT:          float = 0.0    # GPS disabled; always near-zero trust
TRUST_EPSILON:              float = 1e-6   # matches TrustWeights::EPSILON in C++


# ─────────────────────────────────────────────────────────────────────────────
# Model hyperparameters
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ModelConfig:
    n_features:    int   = len(FEATURE_COLS)   # 11
    seq_len:       int   = 20     # 2 s @ 10 Hz sync rate
    d_model:       int   = 64     # Transformer hidden dimension
    n_heads:       int   = 4      # attention heads  (d_model % n_heads == 0)
    n_layers:      int   = 2      # TransformerEncoder stacked layers
    dropout:       float = 0.1


# ─────────────────────────────────────────────────────────────────────────────
# Training hyperparameters
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class TrainConfig:
    # ── Data ────────────────────────────────────────────────────────────────
    data_dir:          str   = "~/fgo_training_data"
    stats_path:        str   = "~/fgo_training_data/norm_stats.json"
    checkpoint_dir:    str   = "~/fgo_training_data/checkpoints"

    val_fraction:      float = 0.15   # fraction of rows held out for validation
    nan_fill:          float = 0.0    # value substituted for NaN feature cells

    # ── Sequence ────────────────────────────────────────────────────────────
    seq_len:           int   = 20
    step:              int   = 1      # sliding window stride (1 = every row)

    # ── Optimisation ────────────────────────────────────────────────────────
    epochs:            int   = 60
    batch_size:        int   = 256
    lr:                float = 3e-4
    weight_decay:      float = 1e-4
    lr_scheduler:      str   = "cosine"   # "cosine" or "step"
    lr_step_size:      int   = 20         # used when lr_scheduler == "step"
    lr_gamma:          float = 0.5        # used when lr_scheduler == "step"

    # ── Loss weights ────────────────────────────────────────────────────────
    # Total loss = lambda_error * L_error
    #            + lambda_trust * GT_weight(pos_err) * L_trust
    #            + lambda_yaw   * L_yaw
    lambda_error:      float = 1.0    # weight for error-prediction MSE
    lambda_trust:      float = 0.5    # weight for trust pseudo-label MSE
    lambda_yaw:        float = 0.1    # weight for yaw-error auxiliary loss
    # GT weight = sigmoid(pos_err_m / err_scale)
    # err_scale controls at what error magnitude trust supervision "turns on":
    #   err_scale=0.05 → 50% weight at 5 cm error (tight)
    #   err_scale=0.10 → 50% weight at 10 cm error (default)
    err_scale:         float = 0.10

    # ── Misc ────────────────────────────────────────────────────────────────
    seed:              int   = 42
    num_workers:       int   = 2
    log_interval:      int   = 10     # print loss every N batches
