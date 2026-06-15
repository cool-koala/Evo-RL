#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
conda_sh="${CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
conda_env="${CONDA_ENV:-evo-rl-ros2-jazzy}"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

# ROS and conda setup scripts aren't safe under `set -u`.
set +u
if [[ -f "$conda_sh" ]]; then
    # shellcheck disable=SC1090
    source "$conda_sh"
    conda activate "$conda_env"
fi

if [[ -f "$ros_setup" ]]; then
    # shellcheck disable=SC1090
    source "$ros_setup"
fi
set -u

cd "$repo_root"
exec python scripts/cobot_magic_reset_pose.py "$@"
