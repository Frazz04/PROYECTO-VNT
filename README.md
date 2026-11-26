# F1Tenth Autonomous Navigation: Reactive Obstacle Avoidance & Telemetry Pipeline

## Project Presentation

| Field | Details |
| :--- | :--- |
| **Institution** | Escuela Superior Politécnica del Litoral (ESPOL) |
| **Program / Area** | Mechatronics / Control & Autonomous Robotics |
| **Author** | Alain |
| **Simulation Environment** | ROS 2 Humble — `f1tenth_gym_ros` (`Sakhir_obs_map`) |
| **Control Strategy** | Purely Reactive Follow the Gap (FTG) + Disparity Extender |
| **Scope** | Part 1 — Single-Agent High-Speed Navigation with Static Obstacles & Data Logging |

---

## Project Media & Telemetry Datasets (Part 1)

* **YouTube Demonstration (Part 1 — Single Vehicle with Static Obstacles):** [Watch Video on YouTube](YOUR_YOUTUBE_LINK_PART_1)
* **Google Drive Repository (Rosbag & Exported CSV Files):** [Download Rosbag & CSV Data on Google Drive](YOUR_GOOGLE_DRIVE_LINK)
  * *Included files:* `rosbag_part1/` (`.db3` + `metadata.yaml`) and converted CSV tables (`odometry.csv`, `lidar.csv`, `velocity.csv`).

---

## 1. Introduction

Autonomous vehicles operating in high-speed racing environments must perceive surroundings, plan collision-free trajectories, and execute steering and throttle commands within milliseconds. Unlike low-speed mobile robots that rely on pre-built global maps and Simultaneous Localization and Mapping (SLAM), autonomous racing platforms require reactive local controllers capable of adapting immediately to unmapped obstacles and tight cornering geometries.

This project presents the design, implementation, and validation of a purely reactive navigation system developed in **ROS 2 Humble** for a 1/10th-scale Ackermann-steered racecar inside the **`f1tenth_gym_ros`** physics simulator. In **Part 1**, the autonomous agent (`ego_racecar`) is evaluated on the **Sakhir** circuit modified with **5 static obstacles** (`Sakhir_obs_map`) distributed across straightaways and braking zones.

### Project Objectives
1. **Mapless Reactive Navigation:** Navigate the modified Sakhir track cleanly at speeds up to `11.0 m/s` using solely instantaneous 2D LiDAR scans (`/scan`), without reading the map file or using global path planners.
2. **Reliable Obstacle Evasion:** Prevent corner-clipping collisions against static blocks and hairpin walls by combining a **Disparity Extender** with a dynamic **Safety Bubble** and adaptive gap centering.
3. **Real-Time Terminal & RViz Telemetry:** Display live lap timing, odometry state (`X`, `Y`, `Yaw`, linear and angular velocities), LiDAR clearances, and commanded control signals in the terminal alongside a 3D HUD marker in RViz2.
4. **Data Acquisition & Post-Processing Pipeline:** Record full simulation runs via `ros2 bag` (`/ego_racecar/odom`, `/scan`, `/drive`) and convert the raw `.db3` recordings into structured `.csv` files for quantitative performance analysis.

---

## 2. Methodology

The navigation and data-logging architecture is built around four core components:

1. **Purely Reactive Local Navigation (No Map Dependency):**
   The node (`reactive_node.py`) never reads the occupancy grid map. Every control cycle is computed directly from instantaneous `sensor_msgs/LaserScan` ranges on `/scan`, making the system adaptable to unknown obstacle layouts.
2. **Follow the Gap (FTG) with Disparity Extender:**
   Standard Follow-the-Gap algorithms treat LiDAR rays as point measurements from the sensor center, which can cause wheel collisions when clipping obstacle corners. By combining a **Disparity Extender** (expanding sharp depth jumps by half the vehicle's safety width) with a **Safety Bubble** around the closest obstacle, the vehicle carves collision-free corridors around all 5 static blocks.
3. **Adaptive Target Selection & Multi-Constraint Velocity Control:**
   The controller dynamically adjusts its aiming weight between the geometric center and the deepest point of the widest free corridor depending on passage width, applies a steering deadband to eliminate high-speed oscillations on straights, and computes linear speed using continuous constraints for forward clearance, steering angle, and lateral proximity.
4. **Real-Time Telemetry & Rosbag-to-CSV Pipeline:**
   Live vehicle state (odometry pose, LiDAR clearances, linear/angular velocities, and lap times) is printed to the terminal at `4 Hz` and rendered in RViz2 via `visualization_msgs/Marker`. Simultaneously, `ros2 bag record` captures `/ego_racecar/odom`, `/scan`, and `/drive`, which are deserialized into structured `.csv` files using `scripts/bag_to_csv.py`.

---

## 3. Repository File Hierarchy

```text
gap_follow/
├── config/
│   └── sim.yaml                 # Simulator configuration (Sakhir_obs_map, num_agent: 1, initial pose)
├── gap_follow/
│   ├── __init__.py              # Python package initializer
│   └── reactive_node.py         # Main ROS 2 node: LiDAR processing, FTG control, lap timer & telemetry
├── maps/
│   ├── Sakhir_map.png           # Base Sakhir circuit occupancy grid
│   ├── Sakhir_obs_map.png       # Modified Sakhir map with 5 static obstacles (0.40 m x 0.40 m)
│   └── Sakhir_obs_map.yaml      # Map metadata (resolution in m/px, origin coordinates, thresholds)
├── resource/
│   └── gap_follow               # Ament index marker file for ROS 2 package discovery
├── scripts/
│   └── bag_to_csv.py            # Offline converter from ROS 2 bag (.db3) to odometry/lidar/velocity CSVs
├── test/                        # Code style and linter tests (flake8, pep257, copyright)
├── package.xml                  # ROS 2 package manifest and dependencies (rclpy, sensor_msgs, etc.)
├── setup.cfg                    # Script installation directory configuration
├── setup.py                     # Package entry points ('ros2 run gap_follow reactive_node')
└── README.md                    # Technical documentation
```

---

## 4. Step-by-Step Breakdown of `reactive_node.py`

The `ReactiveFollowGap` class executes the following pipeline on every incoming LiDAR scan (`lidar_callback`) and odometry update (`odom_callback`):

### Step 1: Node Initialization & ROS 2 Interfaces (`__init__`)
* Declares configurable ROS 2 parameters (`scan_topic`, `drive_topic`, `odom_topic`, `max_speed = 11.0`, `is_ego = True`).
* Subscribes to `/scan` (`LaserScan`) and `/ego_racecar/odom` (`Odometry`).
* Publishes Ackermann control commands to `/drive` (`AckermannDriveStamped`) and RViz HUD text to `/lap_telemetry_marker` (`Marker`).
* Starts a `0.25 s` (`4 Hz`) timer calling `display_telemetry()` to print real-time diagnostics in the terminal.

### Step 2: LiDAR Preprocessing (`preprocess_lidar`)
* **Sanitization & Lookahead Horizon:** Replaces `NaN`/`Inf` returns and clips maximum range readings to **`18.0 m`** so the vehicle does not over-accelerate on long straights before detecting upcoming braking zones.
* **Field-of-View Filtering:** Zeroes out rays outside $[-1.30, +1.30]\text{ rad}$ ($\approx \pm 74.5^\circ$), ignoring rear side walls and focusing strictly on the forward hemisphere.

### Step 3: Disparity Extender (`apply_disparity_extender`)
* Scans adjacent LiDAR rays for abrupt depth jumps ($\vert{}\Delta r\vert{} > 0.5\text{ m}$), which indicate obstacle edges or hairpin walls.
* Projects a lateral safety half-margin ($m = 0.28\text{ m}$) over an angular arc:
  $$\theta_{\text{ext}} = \arctan\left(\frac{0.28}{\max(r_{\text{close}}, 0.1)}\right)$$
* Overwrites neighboring indices with $r_{\text{close}}$, virtually inflating obstacles so the chassis clears corners safely.

### Step 4: Safety Bubble
* Identifies the single closest valid obstacle return (`closest_idx`) and sets a radius of **`44` indices** around it to `0.0`, creating an immediate exclusion zone.

### Step 5: Maximum Gap Selection & Adaptive Target Aiming
* Splits all remaining non-zero rays into contiguous free-space segments and selects the widest gap (`max_gap`).
* Computes the gap's geometric midpoint (`center_idx`) and the median index of its deepest rays (`deepest_idx`).
* Blends both indices adaptively:
  * **Narrow obstacle corridors (`len(max_gap) < 160` rays):** Uses `65% center + 35% deepest` to keep balanced lateral clearance between walls and static blocks.
  * **Wide open corners (`len(max_gap) >= 160` rays):** Uses `35% center + 65% deepest` to hit fast racing apexes.

### Step 6: Steering Deadband
* Converts `best_idx` into a steering angle (`raw_angle`). If $\vert{}\text{angle}\vert{} < 4.0^\circ$, it is snapped to `0.0 rad` to eliminate high-frequency steering oscillations ("worm effect") on straightaways.

### Step 7: Multi-Constraint Velocity Profiling & Angular Velocity Estimation
* Computes `effective_dist = min(target_distance, front_dist)` using a narrow $\pm 2.0^\circ$ frontal cone for early braking before corners.
* Calculates the final speed command (`3.4 m/s` to `11.0 m/s`) as the minimum of three limits (`min(v_dist, v_angle, v_prox)`):
  1. **Forward Clearance (`v_dist`):** Non-linear ramp ($\text{ratio}^{1.3}$) scaling from `3.4 m/s` ($\le 2.5\text{ m}$) up to `11.0 m/s` ($\ge 10.5\text{ m}$).
  2. **Steering Angle (`v_angle`):** Reduces speed linearly (`10.2 - 13.0 * |angle|`) during sharp turns.
  3. **Lateral Proximity (`v_prox`):** Applies extra damping when any object comes within `0.70 m` of the vehicle.
* Estimates commanded yaw rate using the kinematic bicycle model ($L = 0.3302\text{ m}$):
  $$\omega = \frac{v}{L} \tan(\delta)$$

### Step 8: Hysteresis Lap Counter & Live Telemetry (`odom_callback` & `display_telemetry`)
* Converts quaternion orientation to yaw angle ($\psi$) and tracks real linear/angular twist velocities.
* Uses spatial hysteresis (`> 5.0 m` departure threshold, `< 2.5 m` finish-line radius) to count completed laps accurately and display `[LAPS]`, `[ODOMETRY]`, `[LIDAR]`, and `[VELOCITY]` blocks in English.

---

## 5. Build, Execution & Data Export Workflow (Part 1)

### 1. Build the Workspace
```bash
cd ~/F1Tenth-Repository
colcon build --packages-select f1tenth_gym_ros gap_follow
source install/setup.bash
```

### 2. Run Simulation, Rosbag Recording & Reactive Controller
Open **3 terminals** and run in order:

* **Terminal 1 — Launch Simulator (`Sakhir_obs_map`, 1 agent):**
  ```bash
  cd ~/F1Tenth-Repository && source install/setup.bash
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  ```
* **Terminal 2 — Record Rosbag (`Odometry`, `LiDAR`, `Velocity`):**
  ```bash
  mkdir -p ~/F1Tenth-Repository/bags && cd ~/F1Tenth-Repository/bags
  source ~/F1Tenth-Repository/install/setup.bash
  ros2 bag record -o rosbag_part1 /ego_racecar/odom /scan /drive
  ```
* **Terminal 3 — Launch Reactive Navigation Node:**
  ```bash
  cd ~/F1Tenth-Repository && source install/setup.bash
  ros2 run gap_follow reactive_node
  ```

### 3. Convert Rosbag to CSV
After stopping the nodes (`Ctrl + C` in Terminal 3, then `Ctrl + C` in Terminal 2), run:
```bash
cd ~/F1Tenth-Repository/bags
python3 ~/F1Tenth-Repository/src/gap_follow/scripts/bag_to_csv.py rosbag_part1 csv_part1
```
This generates `odometry.csv`, `lidar.csv`, and `velocity.csv` inside `csv_part1/`.
