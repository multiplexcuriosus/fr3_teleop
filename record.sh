#!/usr/bin/env bash
set -euo pipefail

# -----------------------------
# Defaults
# -----------------------------
OUT_DIR="data/bags"
PREFIX="real_il"

# parse args like target_dir=... prefix=...
for arg in "$@"; do
  case "$arg" in
    target_dir=*)
      OUT_DIR="${arg#*=}"
      ;;
    prefix=*)
      PREFIX="${arg#*=}"
      ;;
    *)
      echo "Unknown argument: $arg"
      echo "Usage: $0 [target_dir=PATH] [prefix=NAME]"
      exit 1
      ;;
  esac
done

# -----------------------------
# Output path
# -----------------------------
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_PATH="${OUT_DIR}/${PREFIX}_${TIMESTAMP}"

echo "Recording to: $BAG_PATH"
mkdir -p "$OUT_DIR"

# -----------------------------
# Topic selection
# -----------------------------
# Keep only topics that are actually relevant for imitation-learning data:
# - episode start/stop control
# - teleop command stream
# - robot proprioception / state
# - gripper state
# - RGB image
# - event image
#
# Excluded on purpose:
# - RViz / MoveIt visualization topics
# - tf / tf_static (usually unnecessary if calibration is fixed elsewhere)
# - debug topics / transition_event
# - compressed image variants
# - haptic feedback-only topics unless you explicitly want raw operator device data
#
TOPICS=(
  # Infra
  /episode/control 
  /camera/camera/color/camera_info

  # Action
  /cartesian_cmd/twist
  /teleop/gripper_cmd

  # Observation
  /camera/camera/color/image_raw
  /openmv_cam/image

  # Proprioception
  /joint_states
  /right_fr3/joint_states
  /right_franka_gripper/joint_states

  /right_franka_robot_state_broadcaster/current_pose
  /right_franka_robot_state_broadcaster/measured_joint_states
  /right_franka_robot_state_broadcaster/robot_state
)

# -----------------------------
# Record
# -----------------------------
ros2 bag record \
  -o "$BAG_PATH" \
  "${TOPICS[@]}"