#!/usr/bin/env bash
# ==============================================================================
# setup_ubuntu.sh — Ubuntu 24.04 (Noble) Environment Setup for Synapse Humanoid
#
# Configures ROS 2 Jazzy, build tools, real-time control, and AI perception
# dependencies. Usable on native Ubuntu 24.04 LTS or inside OrbStack / Docker.
# ==============================================================================

set -euo pipefail

# ── Colors & Logging ─────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${BLUE}[INFO]${RESET}  $*"; }
success() { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; exit 1; }
step()    { echo -e "\n${BOLD}━━━ $* ━━━${RESET}"; }

# ── 1. Check OS Version ──────────────────────────────────────────────────────
step "1. Checking Ubuntu Version"
if [ ! -f /etc/os-release ]; then
  error "Not a Linux system with /etc/os-release"
fi

. /etc/os-release
info "Detected OS: $PRETTY_NAME ($VERSION_CODENAME)"

if [ "$VERSION_CODENAME" != "noble" ]; then
  warn "This script is optimized for Ubuntu 24.04 LTS (Noble). Current: $VERSION_CODENAME"
fi

# ── 2. Configure Repositories & Locale ────────────────────────────────────────
step "2. Configuring Locale and Ubuntu Universe Repository"
sudo apt-get update
sudo apt-get install -y locales curl gnupg lsb-release software-properties-common

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository -y universe

# ── 3. Add ROS 2 Jazzy APT Repository ─────────────────────────────────────────
step "3. Adding ROS 2 Jazzy APT Key and Repository"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update

# ── 4. Install Core Packages & Dependencies ───────────────────────────────────
step "4. Installing ROS 2 Jazzy Desktop, ros2_control and Build Tools"
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-message-filters \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-flake8 \
  libopencv-dev

# ── 5. Initialize rosdep ──────────────────────────────────────────────────────
step "5. Initializing rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update

# ── 6. Setup Shell Environment ────────────────────────────────────────────────
step "6. Configuring ~/.bashrc"
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc"; then
  echo "" >> "$HOME/.bashrc"
  echo "# ROS 2 Jazzy Environment" >> "$HOME/.bashrc"
  echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"
  info "Added 'source /opt/ros/jazzy/setup.bash' to ~/.bashrc"
fi

success "Synapse Humanoid Ubuntu 24.04 environment is fully configured!"
