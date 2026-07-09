# FYDP Cv2: DIY 3D LiDAR Scanner for Search & Rescue

This repository contains the complete firmware, ROS 2 drivers, and map processing software for the **Search and Rescue (SAR) 3D LiDAR Scanner**. The system distributes compute workloads between an **ESP32-S3** microcontroller (motor/IMU drivers), a **Horizon Robotics RDK X5** single-board computer (rover sensor node), and a **Laptop** (base station mapping and visualization).

---

## 🛠 Hardware Configuration & Wiring

### 1. ESP32-S3 to TMC2209 (Stepper Motor Driver)
* **GPIO 4** $\rightarrow$ TMC2209 **STEP**
* **GPIO 5** $\rightarrow$ TMC2209 **DIR**
* **GPIO 6** $\rightarrow$ TMC2209 **EN** (LOW = enabled, active sweep)

### 2. ESP32-S3 to Fermion BNO055 (9-Axis IMU)
* **GPIO 8** $\rightarrow$ BNO055 **SDA** (I2C Data)
* **GPIO 9** $\rightarrow$ BNO055 **SCL** (I2C Clock)
* **3.3V / GND** $\rightarrow$ BNO055 **VCC / GND**

---

## 💾 Firmware Setup (PlatformIO)

The firmware is located in `/esp32_stepper/` and manages:
1. Reading base quaternion orientation at 50 Hz.
2. Motion gating (detecting base movement via BNO055 gyroscope magnitude `gyro_mag > 5.0 deg/sec` to avoid linear accelerometer gravity drift).
3. Back-and-forth tilt sweeping using a NEMA-17 motor controlled via TMC2209.
4. Serial reporting format: `IMU:qw,qx,qy,qz`, `STEP:angle`, and `MOVING:0/1`.

### Flashing the Firmware:
1. Connect the ESP32-S3 to your laptop via USB.
2. Enter bootloader mode: Press and hold the **BOOT** button, press **EN/RST** once, and release **BOOT**.
3. Flash the code:
   ```bash
   cd esp32_stepper
   ~/.platformio/penv/bin/pio run --target upload --upload-port /dev/ttyACM0
   ```
4. Press the **EN/RST** button once to boot the microcontroller.

---

## 🌐 Network Setup (Cyclone DDS)

Since the RDK X5 runs **ROS 2 Humble** (Ubuntu 22.04) and the Laptop runs **ROS 2 Jazzy** (Ubuntu 24.04), they must use **Cyclone DDS** to communicate reliably over WiFi:

1. **Install Cyclone DDS on both machines:**
   * **RDK X5:** `sudo apt install ros-humble-rmw-cyclonedds-cpp`
   * **Laptop:** `sudo apt install ros-jazzy-rmw-cyclonedds-cpp`

2. **Add to `~/.bashrc` on both machines:**
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
   *Then reload your terminals:* `source ~/.bashrc`

---

## 🚀 Running the System

### Step 1: Start Hardware Drivers on RDK X5
Connect your RPLiDAR C1 and ESP32-S3 to the RDK X5 USB ports, SSH into the RDK, and launch:
```bash
ros2 launch fypd_cv2 rover_launch.py \
    serial_port_esp32:=/dev/ttyACM0 \
    serial_port_lidar:=/dev/ttyUSB0
```
*Configures the RPLiDAR C1 scan rate to **15 Hz** and begins the stepper sweep.*

### Step 2: Start Map Processing on Laptop
Run the voxel point cloud accumulator node on your laptop:
```bash
cd ros2_ws
source install/setup.bash
ros2 launch fypd_cv2 laptop_launch.py \
    enable_motion_gating:=true \
    invert_z:=False
```

### Step 3: Open RViz2 on Laptop
Launch RViz:
```bash
rviz2
```
* **Fixed Frame:** Set to `map`.
* **Add PointCloud2 Display:** Set topic to `/map_3d` with **Decay Time** set to `0` (accumulates points persistently).

---

## 📐 Advanced Scanning Methods

### 1. Gyro-Only Motion Gating
Linear accelerometers suffer from drift and motor vibration noise. The ESP32 evaluates movement solely using the gyroscope's angular velocity magnitude ($>5^\circ\text{/sec}$). During motion, mapping is **paused** (freezing the persistent `/map_3d` to prevent smearing), while the real-time sensor frame orientation updates live in RViz.

### 2. 90-Degree ICP Auto-Alignment
When the scanner transitions from moving to stationary (such as placing it down at a new $90^\circ$ angle), the node triggers an automatic yaw alignment. It runs a correlation search over $\pm 10^\circ$ in steps of $0.5^\circ$ against the existing point cloud map, finding the optimal yaw offset to snap the new scans into alignment.

### 3. Voxel Overwrite Prevention
To prevent double-wall ghosting artifacts from minor IMU yaw drifts, the node implements a density filter. If a voxel's neighbors (within a 3-voxel search radius, $\approx 6\text{ cm}$) are already populated by stable points (points scanned at least 5 times), any slightly offset duplicate points are **automatically rejected**. Scans of previously unmapped blind zones are accepted without restrictions.
