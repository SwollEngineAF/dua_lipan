# Lipan 2.0
Updated by SPBSyahiid 2025/04/30

This repo documents the complete setup of the **Lipan 2.0** robot platform, developed and tested using ROS 2 Jazzy, Gazebo Harmonic (`gz sim`), and RViz2 on a **clean install of Ubuntu 24.04 (WSL2)**.

---

## ✅ Prerequisites

- Ubuntu 24.04 (tested on WSL2)
- Installed:
  - ROS 2 Jazzy (`ros-jazzy-desktop`)
  - `gazebo-harmonic` (gz sim)
  - `xacro`, `ros_gz`, `ros-jazzy-ros-gz` packages
- Git and VSCode (optional but recommended)
- Working WSLg + OpenGL for GUI support (RViz2, Gazebo)

---

## 🧰 Setup Instructions

### 1. Install ROS 2 Jazzy
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop
```
Add to your `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
```
Apply the changes:
```bash
source ~/.bashrc
```

### 2. Install Gazebo Harmonic (gz sim)
```bash
sudo apt install gz-sim8 gz-gui8
```
Also install additional dependencies:
```bash
sudo apt install ros-jazzy-xacro ros-jazzy-ros-gz
```

### 3. Create and Initialize ROS 2 Workspace
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

### 4. Clone the Articubot Repo
```bash
git clone https://github.com/SwollEngineAF/lipan.git lipan
cd ~/dev_ws
```

### 5. Build the Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```
Add to your `~/.bashrc` for future sessions:
```bash
source ~/dev_ws/install/setup.bash
```

---

## 🚀 Launching the Simulation
After a successful build:
```bash
ros2 launch lipan launch_sim_simple.launch.py
```
If RViz2 and Gazebo do not appear automatically, you can launch them manually:
```bash
gz sim
rviz2
```

---

## 🔧 Troubleshooting

- **Missing packages**: Install using `sudo apt install ros-jazzy-<package-name>`
- **Git push issues**:
  ```bash
  git remote set-url origin https://github.com/SwollEngineAF/dua_lipan.git
  ```
- **Gazebo or RViz2 GUI not loading (WSL)**:
  ```bash
  export DISPLAY=:0
  export LIBGL_ALWAYS_INDIRECT=1
  ```
  Add these to your `~/.bashrc` if needed

---

## 📆 Full Roadmap – Lipan 2.0

### 🌀 Iteration 1: Simulation Foundations (April 7 – April 13)
- Set up ROS 2 Humble on dev machine
- Clone lipan and run in Gazebo
- Replace Gazebo world with palm/obstacles layout
- Simulate RGB camera and verify image stream
- Detect dummy FFB using OpenCV in sim images
- Visualize detection markers and TFs in RViz2

### 🌀 Iteration 2: Sim Integration (April 21 – April 27)
- Add manipulator arm to URDF
- Configure joint controllers in ROS 2
- Write node to convert pose to motion command
- Simulate full detect → plan → move pipeline
- Visualize joint states and full motion in RViz2

### 🌀 Iteration 3: Hardware Setup (May 5 – May 11)
- Flash Raspberry Pi OS and install ROS 2
- Install DepthAI SDK and test OAK-D
- Install and launch depthai-ros
- Flash Arduino and test basic serial comms
- Build teleop_control_node.py
- Test teleop commands on real actuator

### 🌀 Iteration 4: Real-World Integration (May 19 – May 25)
- Mount hardware on Lipan and wire safely
- Run real-time FFB detection on OAK-D
- Convert detected pose to actuator command
- Switch between teleop and auto mode
- Field test: detect → target → move

---

## ✅ Status

- [x] Fresh ROS 2 Jazzy setup ✅
- [x] Gazebo Harmonic running ✅
- [x] RViz2 visualization ✅
- [x] GitHub repo integration ✅
- [x] Project roadmap created ✅

---

## 🧩 Hardware Add-ons (Adhoc Integration)

As of May 2025, the Lipan 2.0 platform includes tested modules beyond the base simulation stack. These components were integrated and tested independently in real-world setups.

### 📸 Pixy2 Vision Sensor

* Connected to **Raspberry Pi 5** via USB.
* Streams object coordinates over serial (used for signature-based detection).
* Used in: `arduino/sketch_may26d/sketch_may26d.ino`, `pixy_stream.py`

### 🦾 Robot Arm (PCA9685 Servo Driver)

* Controlled using **Adafruit PCA9685** via I2C.
* Four active channels for: Base, Left Arm, Right Arm, Gripper.
* Calibrated PWM ranges defined in `arduino/Arm/src/main.cpp`
* Commands can be sent via serial: e.g., `B:500`, `L:450`, `G:1`.

### ⚙️ Differential Drive (HW-130)

* Implemented as basic motor control with `digitalWrite()` test on Arduino Uno.
* Setup stored in: `arduino/DiffDRive/`
* Placeholder for real-world DC motor control testing (PWM/direction).

### 🛠 Example Scripts

* `pixy_stream.py` – Serial stream reader for Pixy2.
* `servo_gui.py` – Tkinter-based servo tester GUI.
* `sketch_may26[a–e].ino` – Standalone sketches for debugging servos and Pixy2.

---

## ✍️ Author
**Syahiid Rasidi** – [@SwollEngineAF](https://github.com/SwollEngineAF)
