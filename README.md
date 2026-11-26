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

## 4. Complete Code Walkthrough of `reactive_node.py`

Below is the complete source code of `reactive_node.py` divided sequentially into functional blocks, explaining how each section operates inside the ROS 2 control loop.

### 4.1 Libraries, ROS 2 Parameters & State Initialization (`__init__`)

```python
import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')

        # Configurable parameters for Ego (Part 1 & 2) and Opponent (Part 2)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('opp_odom_topic', '/opp_racecar/odom')
        self.declare_parameter('max_speed', 11.0)
        self.declare_parameter('is_ego', True)

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        opp_odom_topic = self.get_parameter('opp_odom_topic').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.is_ego = self.get_parameter('is_ego').get_parameter_value().bool_value

        self.subscription = self.create_subscription(LaserScan, scan_topic, self.lidar_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        if self.is_ego:
            self.opp_odom_sub = self.create_subscription(Odometry, opp_odom_topic, self.opp_odom_callback, 10)
            self.marker_pub = self.create_publisher(Marker, '/lap_telemetry_marker', 10)
            self.timer = self.create_timer(0.25, self.display_telemetry)

        # Start state and synchronized stopwatch
        self.start_x = None
        self.start_y = None
        self.opp_start_x = None
        self.opp_start_y = None
        self.ego_moving = False
        self.opp_moving = False
        self.opp_seen = False
        self.race_started = False

        self.lap_count = 0
        self.lap_start_time = None
        self.last_lap_time = 0.0
        self.best_lap_time = float('inf')
        self.left_start_zone = False

        # Live telemetry variables (Odometry, LiDAR, and Velocities)
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.odom_lin_vel = 0.0
        self.odom_ang_vel = 0.0

        self.lidar_front_dist = 0.0
        self.lidar_min_dist = 0.0
        self.lidar_target_dist = 0.0

        self.cmd_lin_vel = 0.0
        self.cmd_steering = 0.0
        self.cmd_ang_vel = 0.0
        self.wheelbase = 0.3302  # Standard F1Tenth wheelbase (m)
```

* **What this block does:**
  * Imports `rclpy` for ROS 2 node execution, `numpy` for fast vectorized LiDAR array operations, and standard ROS 2 message definitions (`LaserScan`, `AckermannDriveStamped`, `Odometry`, `Marker`).
  * Declares runtime parameters (`scan_topic`, `drive_topic`, `odom_topic`, `max_speed`, `is_ego`) so the exact same node executable can be reconfigured without modifying source code.
  * Creates subscriptions to `/scan` and `/ego_racecar/odom`, a publisher on `/drive`, a 3D HUD marker publisher on `/lap_telemetry_marker`, and a `0.25 s` (`4 Hz`) periodic timer for terminal telemetry output.
  * Initializes state variables for lap timing, spatial hysteresis, live sensor readings, and the physical F1Tenth wheelbase ($L = 0.3302\text{ m}$).

---

### 4.2 Odometry Processing, Synchronized Start & Hysteresis Lap Counter (`opp_odom_callback` & `odom_callback`)

```python
    def opp_odom_callback(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        self.opp_seen = True

        if self.opp_start_x is None:
            self.opp_start_x = ox
            self.opp_start_y = oy
            return

        if not self.opp_moving and math.hypot(ox - self.opp_start_x, oy - self.opp_start_y) > 0.15:
            self.opp_moving = True

    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        # Extract Yaw orientation from quaternion and actual twist velocities
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.curr_x = curr_x
        self.curr_y = curr_y
        self.odom_lin_vel = msg.twist.twist.linear.x
        self.odom_ang_vel = msg.twist.twist.angular.z

        if self.start_x is None:
            self.start_x = curr_x
            self.start_y = curr_y
            return

        if not self.ego_moving and math.hypot(curr_x - self.start_x, curr_y - self.start_y) > 0.15:
            self.ego_moving = True

        # Start stopwatch only once both vehicles move (or ego alone in 1-agent mode)
        if not self.race_started:
            both_ready = self.ego_moving and (self.opp_moving if self.opp_seen else True)
            if both_ready:
                self.race_started = True
                self.lap_start_time = time.time()
            else:
                return

        dist_to_start = math.hypot(curr_x - self.start_x, curr_y - self.start_y)

        if dist_to_start > 5.0:
            self.left_start_zone = True

        if self.left_start_zone and dist_to_start < 2.5:
            now = time.time()
            self.last_lap_time = now - self.lap_start_time
            if self.last_lap_time < self.best_lap_time:
                self.best_lap_time = self.last_lap_time
            self.lap_count += 1
            self.lap_start_time = now
            self.left_start_zone = False
            if not self.is_ego:
                self.get_logger().info(f"[OPP] Lap {self.lap_count} completed in {self.last_lap_time:.2f} s")
```

* **What this block does:**
  * **Pose & Velocity Extraction:** Reads the vehicle's $(x, y)$ position, converts the quaternion orientation $(x, y, z, w)$ into planar heading angle $\text{Yaw} = \text{atan2}(2(wz + xy), 1 - 2(y^2 + z^2))$, and stores real linear ($v_x$) and angular ($\omega_z$) twist velocities.
  * **Motion-Triggered Stopwatch:** Records the initial starting coordinates $(x_0, y_0)$ and waits until the vehicle moves more than `0.15 m` before starting `lap_start_time`, avoiding idle waiting time before launch.
  * **Hysteresis Lap Detection:** Prevents false double-counting at the start line by requiring the vehicle to travel farther than `5.0 m` (`left_start_zone = True`) before re-entering a `2.5 m` radius increments `lap_count` and updates `last_lap_time` and `best_lap_time`.

---

### 4.3 Real-Time Terminal Telemetry & RViz2 HUD Marker (`display_telemetry`)

```python
    def display_telemetry(self):
        current_lap_time = (time.time() - self.lap_start_time) if (self.race_started and self.lap_start_time is not None) else 0.0
        best_str = f"{self.best_lap_time:.2f} s" if self.best_lap_time != float('inf') else "--.-- s"
        last_str = f"{self.last_lap_time:.2f} s" if self.lap_count > 0 else "--.-- s"

        telemetry_block = (
            f"\n================================================================================\n"
            f"[LAPS]     Lap: {self.lap_count} | Current: {current_lap_time:.2f} s | Last: {last_str} | Best: {best_str}\n"
            f"[ODOMETRY] X: {self.curr_x:6.2f} m | Y: {self.curr_y:6.2f} m | Yaw: {self.curr_yaw:5.2f} rad | "
            f"Lin_vel: {self.odom_lin_vel:5.2f} m/s | Ang_vel: {self.odom_ang_vel:5.2f} rad/s\n"
            f"[LIDAR]    Front: {self.lidar_front_dist:5.2f} m | Min: {self.lidar_min_dist:5.2f} m | "
            f"Target: {self.lidar_target_dist:5.2f} m\n"
            f"[VELOCITY] Cmd Linear: {self.cmd_lin_vel:5.2f} m/s | Cmd Angular: {self.cmd_ang_vel:5.2f} rad/s | "
            f"Steering: {self.cmd_steering:5.2f} rad"
        )
        print(telemetry_block, flush=True)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lap_counter"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = -4.0
        marker.pose.position.y = -12.0
        marker.pose.position.z = 2.0
        marker.scale.z = 3.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.text = (
            f"LAP: {self.lap_count}\n"
            f"TIME: {current_lap_time:.2f} s\n"
            f"LAST: {last_str}\n"
            f"BEST: {best_str}"
        )
        self.marker_pub.publish(marker)
```

* **What this block does:**
  * Prints four structured diagnostic lines (`[LAPS]`, `[ODOMETRY]`, `[LIDAR]`, and `[VELOCITY]`) to the terminal every `0.25 s` with `flush=True` for immediate live monitoring.
  * Publishes a `TEXT_VIEW_FACING` `visualization_msgs/Marker` anchored in the `map` frame so lap counts and stopwatch times are visible directly inside RViz2 during recording.

---

### 4.4 Disparity Extender (`apply_disparity_extender`)

```python
    def apply_disparity_extender(self, ranges, angle_inc):
        orig = ranges.copy()
        extended = ranges.copy()
        n = len(orig)
        disparity_threshold = 0.5
        car_half_margin = 0.28

        for i in range(n - 1):
            diff = orig[i + 1] - orig[i]
            if abs(diff) > disparity_threshold:
                r_close = min(orig[i], orig[i + 1])
                arc_angle = math.atan2(car_half_margin, max(r_close, 0.1))
                extend_indices = int(arc_angle / angle_inc)

                if diff > 0:
                    end_idx = min(n, i + 1 + extend_indices)
                    extended[i + 1:end_idx] = np.minimum(extended[i + 1:end_idx], r_close)
                else:
                    start_idx = max(0, i + 1 - extend_indices)
                    extended[start_idx:i + 1] = np.minimum(extended[start_idx:i + 1], r_close)

        return extended
```

* **What this block does:**
  * Iterates through consecutive LiDAR beams to detect depth discontinuities larger than `0.5 m` (`disparity_threshold`), which mark the sharp edges of static obstacles or track corners.
  * Computes how many angular ray indices correspond to half the vehicle's width plus safety clearance (`car_half_margin = 0.28 m`) at distance $r_{\text{close}}$:
    $$\theta_{\text{ext}} = \arctan\left(\frac{0.28}{\max(r_{\text{close}}, 0.1)}\right), \quad N_{\text{ext}} = \left\lfloor \frac{\theta_{\text{ext}}}{\Delta\theta} \right\rfloor$$
  * Overwrites adjacent far rays with $r_{\text{close}}$, virtually expanding obstacles in polar space so the vehicle's wheels never clip an edge.

---

### 4.5 LiDAR Sanitization & Field-of-View Masking (`preprocess_lidar`)

```python
    def preprocess_lidar(self, ranges, angle_min, angle_inc):
        proc_ranges = np.array(ranges)
        proc_ranges = np.nan_to_num(proc_ranges, nan=18.0, posinf=18.0, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0.0, 18.0)

        proc_ranges = self.apply_disparity_extender(proc_ranges, angle_inc)

        angles = angle_min + np.arange(len(proc_ranges)) * angle_inc
        blind_spots = np.where(np.abs(angles) > 1.30)[0]
        proc_ranges[blind_spots] = 0.0

        return proc_ranges, angles
```

* **What this block does:**
  * Converts raw scan lists into a NumPy array, sanitizes `NaN`/`Inf` values, and caps the maximum lookahead horizon at **`18.0 m`** to prevent excessive speed buildup on long straights prior to sharp hairpins.
  * Runs `apply_disparity_extender` across the entire scan before masking out rays outside $[-1.30, +1.30]\text{ rad}$ ($\approx \pm 74.5^\circ$), restricting navigation decisions strictly to the forward hemisphere.

---

### 4.6 Safety Bubble & Adaptive Gap Selection (`lidar_callback` — Part A)

```python
    def lidar_callback(self, data):
        proc_ranges, angles = self.preprocess_lidar(data.ranges, data.angle_min, data.angle_increment)

        if np.all(proc_ranges == 0.0):
            return

        # Narrow frontal cone (±2.0 deg) for early braking at end of straights
        center_cone = proc_ranges[np.abs(angles) < math.radians(2.0)]
        front_dist = float(np.min(center_cone)) if len(center_cone) > 0 else 18.0

        ranges = proc_ranges.copy()
        valid_ranges = np.where(ranges > 0.0, ranges, 18.0)
        closest_idx = np.argmin(valid_ranges)
        closest_dist = float(valid_ranges[closest_idx])

        bubble_radius = 44
        start_idx = max(0, closest_idx - bubble_radius)
        end_idx = min(len(ranges), closest_idx + bubble_radius)
        ranges[start_idx:end_idx] = 0.0

        non_zeros = np.where(ranges > 0.0)[0]
        if len(non_zeros) == 0:
            return

        gaps = np.split(non_zeros, np.where(np.diff(non_zeros) != 1)[0] + 1)
        max_gap = max(gaps, key=len)

        center_idx = max_gap[len(max_gap) // 2]
        max_depth = np.max(ranges[max_gap])
        deepest_candidates = np.where(ranges[max_gap] == max_depth)[0]
        deepest_idx = max_gap[deepest_candidates[len(deepest_candidates) // 2]]

        # Adaptive centering: keeps safe margin on narrow gaps (overtaking/obstacles)
        if len(max_gap) < 160:
            best_idx = int(0.65 * center_idx + 0.35 * deepest_idx)
        else:
            best_idx = int(0.35 * center_idx + 0.65 * deepest_idx)
```

* **What this block does:**
  * Measures `front_dist` inside a narrow $\pm 2.0^\circ$ forward cone to detect walls or obstacles directly ahead at the end of straightaways.
  * Finds the closest valid point (`closest_idx`) and zeroes out `44` indices on either side (`bubble_radius`), creating a protective safety bubble around the nearest threat.
  * Splits remaining positive rays into contiguous free-space corridors (`gaps`), selects the widest corridor (`max_gap`), and computes both its geometric midpoint (`center_idx`) and median deepest ray (`deepest_idx`).
  * Blends both references dynamically: in narrow obstacle corridors (`< 160` rays), it weights `65% center + 35% deepest` to keep safe lateral margins; in wide open corners (`>= 160` rays), it shifts to `35% center + 65% deepest` for fast apex entry.

---

### 4.7 Steering Deadband, Multi-Constraint Velocity Control & Publishing (`lidar_callback` — Part B & `main`)

```python
        target_distance = float(proc_ranges[best_idx])
        angle = data.angle_min + (best_idx * data.angle_increment)

        # Overtaking detection: slow car/obstacle ahead (< 6.5m) with open side lane
        is_overtaking = (
            front_dist < 6.5
            and target_distance > (front_dist + 2.5)
            and abs(angle) <= math.radians(20)
        )

        if is_overtaking:
            effective_dist = max(front_dist, min(target_distance, 7.6))
        else:
            effective_dist = min(target_distance, front_dist)

        if abs(angle) < math.radians(4):
            angle = 0.0

        min_speed = 3.4
        max_speed = self.max_speed

        # 1. Forward clearance speed limit
        if effective_dist >= 10.5:
            v_dist = max_speed
        elif effective_dist <= 2.5:
            v_dist = min_speed
        else:
            ratio = (effective_dist - 2.5) / 8.0
            v_dist = min_speed + (max_speed - min_speed) * (ratio ** 1.3)

        # 2. Steering angle speed limit
        if abs(angle) == 0.0:
            v_angle = max_speed
        else:
            v_angle = max(min_speed, min(max_speed, 10.2 - 13.0 * abs(angle)))

        # 3. Lateral proximity speed limit (< 0.70m)
        if closest_dist < 0.70:
            prox_ratio = max(0.0, (closest_dist - 0.30) / 0.40)
            v_prox = 4.2 + (max_speed - 4.2) * prox_ratio
        else:
            v_prox = max_speed

        velocity = min(v_dist, v_angle, v_prox)

        # Store values for terminal telemetry
        self.lidar_front_dist = front_dist
        self.lidar_min_dist = closest_dist
        self.lidar_target_dist = target_distance
        self.cmd_lin_vel = float(velocity)
        self.cmd_steering = float(angle)
        self.cmd_ang_vel = float((velocity / self.wheelbase) * math.tan(angle))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = float(velocity)
        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nNode stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

* **What this block does:**
  * Evaluates `effective_dist` by combining frontal clearance (`front_dist`) with target lane depth (`target_distance`), allowing smooth high-speed bypasses around static blocks when an open side lane (`> front_dist + 2.5 m`) is detected.
  * Applies a **`±4.0°` steering deadband** (`angle = 0.0`) to eliminate high-frequency steering oscillations ("worm effect") on straightaways.
  * Computes linear velocity (`velocity`) between `3.4 m/s` and `11.0 m/s` by taking the most conservative value among three continuous limits:
    1. **`v_dist`:** Non-linear acceleration curve ($\text{ratio}^{1.3}$) governed by forward clearance (`2.5 m` to `10.5 m`).
    2. **`v_angle`:** Linear cornering speed reduction (`10.2 - 13.0 * |angle|`) proportional to steering demand.
    3. **`v_prox`:** Safety damping activated whenever any obstacle enters within `0.70 m` of the chassis.
  * Computes commanded angular velocity via the kinematic Ackermann bicycle model ($\omega = \frac{v}{L}\tan(\delta)$), updates telemetry attributes, publishes `AckermannDriveStamped` to `/drive`, and manages clean node shutdown in `main()`.

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
