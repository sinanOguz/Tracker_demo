
# Visual Tracking and Guidance System

This project implements a real-time vision-based tracking system that tracks a visual target in camera frames.

It integrates:
- OpenCV for object tracking and visualization
- Configurable runtime parameters via `config.h`

---

## Features
- Object tracking with live camera feed or video input
- Debugging overlays (FPS, HUD, selection/track boxes)

---


### Run

```bash
./tracker_app
```

At startup, the app will:

1. Open the default camera (or prompt selection).
2. Start the real-time tracking loop.

---

## Usage

* Mouse click and drag: select tracking region
* Keyboard controls:
  * `p` → pause/unpause
  * `q` → quit
  * `w/e/r` → resize tracker box
  * Arrow keys → nudge tracker box

* Overlays:
  * Green box = tracked target
  * Red box = selection region
  * HUD shows FPS and tracker state

# Tracking Project Setup Guide

This guide provides clear instructions to install dependencies, build, and run the **Tracking** project.


##  **Installation and Build Steps**

### Step 1: Prepare Installation Script

Ensure the provided installation script, `install_dependencies.sh`, exists in your project directory.
If you don't have one, create it using the content provided [above](#).

### Step 2: Grant Execution Permission

Give execution permission to your installation script:

```bash
chmod +x install_dependencies.sh
```

### Step 3: Execute the Script to Install Dependencies

Run the dependency installation script:

```bash
./install_dependencies.sh
```

This script automatically installs:

* `build-essential`
* `cmake`
* `pkg-config`
* `libopencv-dev`

### Step 4: Build Your Project

After dependencies are installed, build the project:

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Step 5: Run the Executable

Run the built executable application:

```bash
./tracker_app
```

---

##  **Verification**

After completing these steps successfully, your **TrackingMonitor** application should run without issues.

