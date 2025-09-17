#!/bin/bash

set -e

echo "Updating package lists..."
sudo apt update

echo "Installing required packages..."
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    libopencv-dev \
    git

echo "Verifying installation..."
cmake --version
pkg-config --modversion opencv4

echo "All dependencies installed successfully!"
