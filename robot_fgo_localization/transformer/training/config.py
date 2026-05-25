"""
Training hyperparameters and schema constants.

All magic numbers live here — nowhere else in the training code.
"""

from dataclasses import dataclass, field
from typing import List


# ─────────────────────────────────────────────────────────────────────────────
# CSV schema
# ─────────────────────────────────────────────────────────────────────────────

# The 10 columns fed as input features to the Transformer.
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
    "slip_metric",      # |gyro_z - angular_vel|    (encoder-IMU angular mismatch)
]

# Primary training target column
LABEL_ERR_COL:  str = "pos_err_m"

# Supplementary label (used as a second loss term with small weight)
LABEL_YAW_COL:  str = "yaw_err_rad"

# ── Per-sensor GT trust label columns ───────────────────────────────────────
# Produced by feature_extractor_node by comparing each sensor's raw measurement
# to Gazebo ground truth.  No hand-crafted thresholds; fully data-driven.
LABEL_LIDAR_ERR_COL:   str = "lidar_pos_error"  # ‖scan_match_pose - GT_pose‖ (m)
LABEL_ENCODER_DRIFT_COL: str = "encoder_drift"  # ‖enc_disp - GT_disp‖ over window (m)
LABEL_IMU_YAW_ERR_COL:  str = "imu_yaw_error"  # |gyro_Δyaw - GT_Δyaw| over window (rad)

# Output index mapping – [encoder, imu, lidar, gps]
TRUST_IDX_ENCODER: int = 0
TRUST_IDX_IMU:     int = 1
TRUST_IDX_LIDAR:   int = 2
TRUST_IDX_GPS:     int = 3


# ─────────────────────────────────────────────────────────────────────────────
# GT trust label conversion parameters
# ─────────────────────────────────────────────────────────────────────────────
# w = 1 / (1 + error / TRUST_ERROR_SCALE)
#
# Interpretation: at error == TRUST_ERROR_SCALE, trust weight = 0.5.
# These are physically motivated — not arbitrary thresholds:
#   TRUST_POS_SCALE  = 0.10 m  →  5 cm error → w ≈ 0.67  (moderate trust)
#                               →  20 cm error → w ≈ 0.33  (low trust)
#   TRUST_YAW_SCALE  = 0.05 rad (≈2.9°) — similar reasoning for angular error
#
# Unlike PSEUDO_*_THRESHOLD, these are scale parameters, not hard gates.
# The model can learn to be more or less sensitive during training.

TRUST_POS_SCALE:   float = 0.10   # metres — for lidar_pos_error and encoder_drift
TRUST_YAW_SCALE:   float = 0.05   # radians — for imu_yaw_error
TRUST_EPSILON:     float = 1e-6   # matches TrustWeights::EPSILON in C++


# ─────────────────────────────────────────────────────────────────────────────
# Model hyperparameters
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ModelConfig:
    n_features:    int   = len(FEATURE_COLS)   # 10
    seq_len:       int   = 50     # 5 s @ 10 Hz sync rate
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
    seq_len:           int   = 50
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
