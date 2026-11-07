#!/usr/bin/env bash
# Native ROS 2 Jazzy on macOS (Apple Silicon) for Mico: RoboStack conda env + MuJoCo + RViz2.
# Builds humanoid_description and humanoid_control into .mac/ so it never clashes with the VM's build/ and install/.
#
#   ./setup_macos.sh                 # env "ros2_jazzy" (created if missing), build
#   ENV_NAME=mico ./setup_macos.sh   # another env name
#
# Then, in every new terminal:
#   conda activate ros2_jazzy && source .mac/install/setup.zsh
#   ros2 launch humanoid_description mico_mujoco.launch.py gait:=true
set -euo pipefail

ENV_NAME="${ENV_NAME:-ros2_jazzy}"
REPO="$(cd "$(dirname "$0")" && pwd)"
info() { printf '\033[34m==>\033[0m %s\n' "$*"; }
fail() { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

[ "$(uname -s)" = "Darwin" ] || fail "this script is for macOS; on Ubuntu use setup_ubuntu.sh"
command -v conda >/dev/null || fail "conda not found: install Miniforge (https://github.com/conda-forge/miniforge)"
CONDA_BASE="$(conda info --base)"
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"
SOLVER=conda
command -v mamba >/dev/null && SOLVER=mamba

if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  info "creating conda env $ENV_NAME from robostack-jazzy (a few GB)"
  $SOLVER create -y -n "$ENV_NAME" -c robostack-jazzy -c conda-forge --override-channels \
    python=3.12 ros-jazzy-desktop ros-jazzy-xacro ros-jazzy-ament-cmake-pytest \
    compilers cmake pkg-config make ninja colcon-common-extensions numpy pytest matplotlib
fi
set +u                                   # conda / ROS activation scripts use unset variables
conda activate "$ENV_NAME"

# libxml2 in the env needs the env's libiconv; the macOS one lacks _libiconv and Python then fails on _scproxy
LIB="$CONDA_PREFIX/lib"
if [ ! -e "$LIB/libiconv.2.dylib" ] && [ -e "$LIB/libiconv.2.dylib.bak" ]; then
  info "restoring $LIB/libiconv.2.dylib"
  mv "$LIB/libiconv.2.dylib.bak" "$LIB/libiconv.2.dylib"
fi
python -c "import _scproxy" || fail "python in $ENV_NAME is broken (libiconv?); try: $SOLVER install -n $ENV_NAME libiconv"

python -c "import mujoco" 2>/dev/null || { info "installing MuJoCo"; python -m pip install mujoco; }
python -c "import mujoco, numpy; print(f'MuJoCo {mujoco.__version__}, numpy {numpy.__version__}')"
for exe in ros2 rviz2 xacro colcon mjpython; do command -v "$exe" >/dev/null || fail "$exe missing in $ENV_NAME"; done

info "building humanoid_description + humanoid_control into .mac/"
cd "$REPO"
colcon --log-base .mac/log build --build-base .mac/build --install-base .mac/install --symlink-install \
  --packages-up-to humanoid_description humanoid_control \
  --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" -DPython3_FIND_STRATEGY=LOCATION

info "checking the gait in MuJoCo (no ROS)"
# shellcheck disable=SC1091
source .mac/install/setup.bash
set -u
python sim/mujoco/check_mujoco.py | head -3

cat <<EOF

Done. In every new terminal:
  conda activate $ENV_NAME && source .mac/install/setup.zsh
  ros2 launch humanoid_description mico_mujoco.launch.py gait:=true    # MuJoCo physics + RViz2
  ros2 launch humanoid_description mico_mujoco.launch.py viewer:=true  # also the MuJoCo viewer
  ros2 launch humanoid_description mico_view.launch.py                 # kinematic playback, no physics
EOF
