#!/usr/bin/env bash

set -e  # Exit immediately if a command exits with a non-zero status.
        # This helps avoid partial installs.

# Usage: ./install_my_repo.sh <REPO_URL>
# Example: ./install_my_repo.sh https://github.com/user/my-python-project.git

REPO_URL="https://github.com/Every-Flavor-Robotics/cmu-16311-tools.git"
MOTORGO_PYTHON_DIR="motorgo-python"
BH1750_DIR="Adafruit_CircuitPython_BH1750"
VL53L4CX_DIR="Adafruit_CircuitPython_VL53L4CX"

# Setup install flags
# If python version is 3.12 or greater, add --break-system-packages flag
EXTRA_PIP_FLAGS=""
# Check if python version is 3.12 or greater
if python -c 'import sys; exit(not (sys.version_info.major == 3 and sys.version_info.minor >= 11))'; then
    # Add --break-system-packages flag to install command
    EXTRA_PIP_FLAGS="--break-system-packages"
fi

# Create a temp directory
TMP_DIR=$(mktemp -d)

# Clone the repo to the temp directory
git clone --recurse-submodules "$REPO_URL" "$TMP_DIR"


# Move into the MotorGo Python dir and install the package
cd "$TMP_DIR/$MOTORGO_PYTHON_DIR"
pip install $EXTRA_PIP_FLAGS .

# Repeart for the BH1750 library
cd "$TMP_DIR/$BH1750_DIR"
pip install $EXTRA_PIP_FLAGS .

# Repeart for the VL53L4CX library
cd "$TMP_DIR/$VL53L4CX_DIR"
pip install $EXTRA_PIP_FLAGS .

# Move back out of the temp directory
cd -

# Remove the cloned temp directory
rm -rf "$TMP_DIR"

echo "Installation complete."
