#!/usr/bin/env bash
set -euo pipefail

# -----------------------------
# Defaults
# -----------------------------
OUT_DIR="data/bags"
PREFIX="mobile_base"

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
# Create helper delete script
# -----------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DELETE_SCRIPT="${SCRIPT_DIR}/delete_latest_bag"

cat <<EOF > "$DELETE_SCRIPT"
#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${OUT_DIR}"
PREFIX="${PREFIX}"

LATEST=\$(ls -dt "\${OUT_DIR}/\${PREFIX}_"* 2>/dev/null | head -n 1 || true)

if [ -z "\$LATEST" ]; then
  echo "No bag found for prefix '\$PREFIX' in '\$OUT_DIR'"
  exit 1
fi

echo "Deleting latest bag: \$LATEST"
rm -rf "\$LATEST"
echo "Done."
EOF

chmod +x "$DELETE_SCRIPT"

# -----------------------------
# Topic selection
# -----------------------------
TOPICS=(
  /episode/control 
  /camera/camera/color/camera_info

  /cartesian_cmd/twist
  /teleop/gripper_cmd

  /camera/camera/color/image_raw
  /openmv_cam/image

  /joint_states
)

# -----------------------------
# Record
# -----------------------------
ros2 bag record \
  -o "$BAG_PATH" \
  "${TOPICS[@]}"