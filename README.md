# F1Tenth Autonomous Navigation: Reactive Obstacle Avoidance, Multi-Agent Overtaking & Telemetry Pipeline

## Project Presentation

| Field | Details |
| :--- | :--- |
| **Institution** | Escuela Superior Politécnica del Litoral (ESPOL) |
| **Program / Area** | Mechatronics / Control & Autonomous Robotics |
| **Author** | Alaín Sánchez|
| **Simulation Environment** | ROS 2 Humble — `f1tenth_gym_ros` (`Sakhir_obs_map`) |
| **Control Strategy** | Purely Reactive Follow the Gap (FTG) + Disparity Extender |
| **Scope** | Part 1 (Single-Agent Static Obstacle Avoidance) & Part 2 (Multi-Agent Overtaking & Dual Telemetry) |

---

## Project Media & Telemetry Datasets

### Part 1 — Single-Agent Navigation with Static Obstacles
* **YouTube Demonstration (Part 1):** [Watch Part 1 Video on YouTube](YOUR_YOUTUBE_LINK_PART_1)
* **Google Drive Repository (Part 1 Rosbag & CSV Files):** [Download Part 1 Rosbag & CSV Data](YOUR_GOOGLE_DRIVE_LINK_PART_1)
  * *Included files:* `rosbag_part1/` (`.db3` + `metadata.yaml`) and converted CSV tables (`odometry.csv`, `lidar.csv`, `velocity.csv`).

### Part 2 — Multi-Agent Dynamic Overtaking (`ego_racecar` vs. `opp_racecar`)
* **YouTube Demonstration (Part 2):** [Watch Part 2 Video on YouTube](YOUR_YOUTUBE_LINK_PART_2)
* **Google Drive Repository (Part 2 Rosbag & CSV Files):** [Download Part 2 Rosbag & CSV Data](YOUR_GOOGLE_DRIVE_LINK_PART_2)
  * *Included files:* `rosbag_part2/` (`.db3` + `metadata.yaml`) and converted CSV tables for both vehicles (`odometry.csv`, `lidar.csv`, `velocity.csv`, `opp_odometry.csv`, `opp_lidar.csv`, `opp_velocity.csv`).

---

## 1. Introduction

Autonomous vehicles operating in high-speed racing environments must perceive surroundings, plan collision-free trajectories, and execute steering and throttle commands within milliseconds. Unlike low-speed mobile robots that rely on pre-built global maps and Simultaneous Localization and Mapping (SLAM), autonomous racing platforms require reactive local controllers capable of adapting immediately to unmapped static obstacles and dynamic agents.

This project presents the design, implementation, and validation of a purely reactive navigation and telemetry system developed in **ROS 2 Humble** for 1/10th-scale Ackermann racecars inside the **`f1tenth_gym_ros`** physics simulator across two progressive stages:

* **Part 1 (Single-Agent Obstacle Avoidance):** Evaluates the autonomous vehicle (`ego_racecar`, up to `11.0 m/s`) on the **Sakhir** circuit modified with **5 static obstacles** (`Sakhir_obs_map`) placed along straights and braking zones.
* **Part 2 (Multi-Agent Overtaking):** Deploys two independent ROS 2 control nodes simultaneously on the track—a high-speed ego racecar (`reactive_node`, `11.0 m/s`) and a slower lead opponent (`opp_reactive_node`, `5.5 m/s`) spawned `5.0 m` ahead—requiring real-time gap selection, lateral safety margins, and non-braking lane-change maneuvers to overtake cleanly without collisions.

### Project Objectives
1. **Mapless Reactive Navigation:** Navigate the modified Sakhir circuit cleanly without reading map files or using global path planners, relying exclusively on instantaneous 2D LiDAR scans.
2. **Static Obstacle Evasion & Dynamic Overtaking:** Prevent corner-clipping collisions and execute safe high-speed overtakes by combining a **Disparity Extender**, a dynamic **Safety Bubble**, adaptive gap centering, and an overtaking velocity override.
3. **Synchronized Multi-Agent Telemetry:** Display real-time lap timing, odometry state (`X`, `Y`, `Yaw`, linear and angular velocities), LiDAR clearances, and commanded velocities in the terminal alongside a 3D HUD marker in RViz2, synchronizing the race stopwatch only once both vehicles initiate motion.
4. **Dual-Agent Data Acquisition & CSV Export Pipeline:** Record single- and multi-agent simulation runs via `ros2 bag` and deserialize all recorded topics into structured `.csv` files for quantitative analysis.

---

## 2. Methodology

The navigation and data-logging architecture is built around five core components:

1. **Purely Reactive Local Navigation (No Map Dependency):**
   Neither vehicle node reads the occupancy grid map. Every control cycle is computed directly from instantaneous `sensor_msgs/LaserScan` ranges, allowing the controller to react identically to fixed walls, static blocks, and moving opponent vehicles.
2. **Follow the Gap (FTG) with Disparity Extender & Safety Bubble:**
   Standard Follow-the-Gap algorithms treat LiDAR beams as point measurements from the sensor center, risking wheel collisions when clipping corners or passing another car. By expanding sharp depth jumps ($\vert{}\Delta r\vert{} > 0.5\text{ m}$) by half the vehicle's safety width (`0.28 m`) via a **Disparity Extender** and zeroing a `44`-ray **Safety Bubble** around the nearest obstacle, the controller carves collision-free corridors.
3. **Adaptive Target Aiming & Overtaking Velocity Logic:**
   * **Adaptive Centering:** In narrow corridors (`< 160` free rays, such as passing static blocks or overtaking the opponent), the target index blends `65% geometric center + 35% deepest ray` to maintain safe clearance on both sides; in open corners (`>= 160` rays), it shifts to `35% center + 65% deepest` for fast apex entry.
   * **Overtaking Detection:** When catching up to the slower opponent (`front_dist < 6.5 m`) while steering into an open adjacent lane (`target_distance > front_dist + 2.5 m`), the controller overrides frontal braking and maintains up to `7.5 m/s` to complete the pass cleanly.
4. **Parameterized Dual-Node Execution & Synchronized Start (Part 2):**
   Both vehicles run from the exact same `reactive_node.py` executable using ROS 2 runtime parameter remapping (`scan_topic`, `drive_topic`, `odom_topic`, `max_speed`, `is_ego`). To guarantee fair lap timing when launching terminals manually, the stopwatch remains at `0.00 s` until both `ego_racecar` and `opp_racecar` move beyond `0.15 m` from their starting grids.
5. **Real-Time Telemetry & Multi-Agent Rosbag-to-CSV Pipeline:**
   Live diagnostic blocks (`[LAPS]`, `[ODOMETRY]`, `[LIDAR]`, `[VELOCITY]`) are printed in English at `4 Hz`, while `ros2 bag record` captures all ego and opponent topics and `scripts/bag_to_csv.py` exports them into separate `.csv` tables.

---

## 3. Repository File Hierarchy

```text
gap_follow/
├── config/
│   ├── sim.yaml                 # Active simulator configuration (Part 2: Sakhir_obs_map, num_agent: 2)
│   ├── sim_part1.yaml           # Saved simulator configuration for Part 1 (num_agent: 1)
│   └── sim_part2.yaml           # Saved simulator configuration for Part 2 (num_agent: 2, sx1: 4.689, sy1: -1.737)
├── gap_follow/
│   ├── __init__.py              # Python package initializer
│   └── reactive_node.py         # Parameterized ROS 2 node for Ego (Part 1 & 2) and Opponent (Part 2)
├── maps/
│   ├── Sakhir_map.png           # Base Sakhir circuit occupancy grid
│   ├── Sakhir_obs_map.png       # Modified Sakhir map with 5 static obstacles (0.40 m x 0.40 m)
│   └── Sakhir_obs_map.yaml      # Map metadata (resolution in m/px, origin coordinates, thresholds)
├── resource/
│   └── gap_follow               # Ament index marker file for ROS 2 package discovery
├── scripts/
│   └── bag_to_csv.py            # Offline ROS 2 bag (.db3) to CSV converter (supports 1 or 2 agents)
├── test/                        # Code style and linter tests (flake8, pep257, copyright)
├── package.xml                  # ROS 2 package manifest and dependencies (rclpy, sensor_msgs, etc.)
├── setup.cfg                    # Script installation directory configuration
├── setup.py                     # Package entry points ('ros2 run gap_follow reactive_node')
└── README.md                    # Technical documentation (Part 1 & Part 2)
```

---

## 4. Multi-Agent ROS 2 Node Architecture (Part 2)

Instead of duplicating source files, Part 2 instantiates two isolated ROS 2 nodes from the same `ReactiveFollowGap` class by overriding ROS 2 parameters at launch:

| Parameter / Interface | Ego Node (`reactive_node`) | Opponent Node (`opp_reactive_node`) | Role in Multi-Agent Simulation |
| :--- | :--- | :--- | :--- |
| **Node Name (`__node`)** | `reactive_node` | `opp_reactive_node` | Prevents node name collisions in the ROS 2 graph |
| **`scan_topic`** | `/scan` | `/opp_scan` | Independent 2D LiDAR sensor streams (`LaserScan`) |
| **`drive_topic`** | `/drive` | `/opp_drive` | Independent Ackermann steering & speed commands |
| **`odom_topic`** | `/ego_racecar/odom` | `/opp_racecar/odom` | Independent pose & velocity feedback (`Odometry`) |
| **`opp_odom_topic`** | `/opp_racecar/odom` | `/opp_racecar/odom` | Monitored by Ego to synchronize stopwatch start |
| **`max_speed`** | `11.0 m/s` | `5.5 m/s` | Speed differential enables Ego to catch and overtake Opp |
| **`is_ego`** | `True` | `False` | Enables `4 Hz` terminal telemetry & RViz2 HUD only on Ego |

---

## 5. Complete Code Walkthrough

### 5.1 Navigation Controller Node (`gap_follow/reactive_node.py`)

#### A. Libraries, ROS 2 Parameters & State Initialization (`__init__`)

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
  * Declares runtime ROS 2 parameters (`scan_topic`, `drive_topic`, `odom_topic`, `opp_odom_topic`, `max_speed`, `is_ego`) so one codebase serves both Part 1 (`ego` only) and Part 2 (`ego` + `opp`).
  * When `is_ego=True`, subscribes to `/opp_racecar/odom` for start synchronization, publishes 3D lap markers to `/lap_telemetry_marker`, and triggers `display_telemetry()` every `0.25 s` (`4 Hz`).

---

#### B. Dual-Agent Odometry, Synchronized Start & Hysteresis Lap Counter (`opp_odom_callback` & `odom_callback`)

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
  * Converts quaternion orientation into planar heading angle ($\text{Yaw}$) and records live twist velocities ($v_x, \omega_z$).
  * **Synchronized Start:** In Part 1 (`opp_seen == False`), the timer begins as soon as `ego_racecar` moves `> 0.15 m`. In Part 2 (`opp_seen == True`), `race_started` waits until **both** `ego_moving` and `opp_moving` become `True`, preventing clock drift while launching terminals.
  * **Spatial Hysteresis:** Requires traveling `> 5.0 m` away from the start line (`left_start_zone = True`) before re-entering a `2.5 m` radius increments `lap_count`.

---

#### C. Live Terminal Telemetry & RViz2 HUD (`display_telemetry`)

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
  * Prints `[LAPS]`, `[ODOMETRY]`, `[LIDAR]`, and `[VELOCITY]` diagnostics to the terminal at `4 Hz` and updates the RViz2 `TEXT_VIEW_FACING` marker on `/lap_telemetry_marker`.

---

#### D. Disparity Extender & LiDAR Preprocessing (`apply_disparity_extender` & `preprocess_lidar`)

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
  * Clips maximum sensor range to **`18.0 m`** to control braking distances on long straights.
  * Expands every depth jump ($\vert{}\Delta r\vert{} > 0.5\text{ m}$)—whether from static obstacles, hairpin corners, or the opponent vehicle's rear bumper—by an angular arc $\theta_{\text{ext}} = \arctan(0.28 / r_{\text{close}})$, then masks angles outside $[-1.30, +1.30]\text{ rad}$ ($\pm 74.5^\circ$).

---

#### E. Safety Bubble, Adaptive Gap Aiming, Overtaking & Velocity Control (`lidar_callback` & `main`)

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
  * Zeroes a `44`-index **Safety Bubble** around `closest_idx`, selects the widest corridor `max_gap`, and blends `center_idx` and `deepest_idx` (`65%/35%` in narrow overtaking/obstacle gaps vs. `35%/65%` in open corners).
  * **Overtaking Override (`is_overtaking`):** When approaching the slower opponent (`front_dist < 6.5 m`) with an open adjacent lane (`target_distance > front_dist + 2.5 m` and `|angle| <= 20°`), `effective_dist` prevents premature braking behind the lead car and maintains `~7.5 m/s` (well above the opponent's `5.5 m/s`) until the pass is completed.
  * Applies a `±4.0°` steering deadband and computes final velocity from `min(v_dist, v_angle, v_prox)` plus kinematic yaw rate $\omega = \frac{v}{L}\tan(\delta)$.

---

### 5.2 Offline Rosbag-to-CSV Converter (`scripts/bag_to_csv.py`)

```python
import sys
import os
import csv
import math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract_scan_metrics(ranges):
    valid = [r for r in ranges if not math.isnan(r) and not math.isinf(r) and r > 0.0]
    n = len(ranges)
    front = ranges[n // 2] if n > 0 else 0.0
    right = ranges[int(n * 0.25)] if n > 0 else 0.0
    left = ranges[int(n * 0.75)] if n > 0 else 0.0
    min_d = min(valid) if valid else 0.0
    max_d = max(valid) if valid else 0.0
    return min_d, front, right, left, max_d


def export_bag_to_csv(bag_path, out_dir):
    os.makedirs(out_dir, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    has_opp = any(t.startswith('/opp_') for t in type_map)
```

* **What this script does:**
  * Opens any ROS 2 bag directory (`rosbag_part1` or `rosbag_part2`) via `rosbag2_py.SequentialReader` and deserializes CDR binary messages into native Python ROS 2 objects (`Odometry`, `LaserScan`, `AckermannDriveStamped`).
  * Automatically detects whether opponent topics (`/opp_racecar/odom`, `/opp_scan`, `/opp_drive`) exist in the bag:
    * **For Part 1:** Generates `odometry.csv`, `lidar.csv`, and `velocity.csv`.
    * **For Part 2:** Generates all 3 Ego CSVs plus `opp_odometry.csv`, `opp_lidar.csv`, and `opp_velocity.csv`.

---

## 6. Build, Execution & Data Export Workflows

### Build the Workspace
```bash
cd ~/F1Tenth-Repository
colcon build --packages-select f1tenth_gym_ros gap_follow
source install/setup.bash
```

---

### Workflow A — Part 1 (Single Vehicle with Static Obstacles)

1. **Activate Part 1 config (`num_agent: 1`) and rebuild simulator:**
   ```bash
   cp ~/F1Tenth-Repository/src/gap_follow/config/sim_part1.yaml ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
   cd ~/F1Tenth-Repository && colcon build --packages-select f1tenth_gym_ros && source install/setup.bash
   ```
2. **Open 3 terminals and run in order:**
   * **Terminal 1 (Simulator):** `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`
   * **Terminal 2 (Rosbag Record):** `cd ~/F1Tenth-Repository/bags && ros2 bag record -o rosbag_part1 /ego_racecar/odom /scan /drive`
   * **Terminal 3 (Ego Controller):** `ros2 run gap_follow reactive_node`
3. **Export Part 1 CSVs** (after pressing `Ctrl + C` in Terminal 3 and Terminal 2):
   ```bash
   cd ~/F1Tenth-Repository/bags
   python3 ~/F1Tenth-Repository/src/gap_follow/scripts/bag_to_csv.py rosbag_part1 csv_part1
   ```

---

### Workflow B — Part 2 (Multi-Agent Overtaking: Ego `11.0 m/s` vs. Opponent `5.5 m/s`)

1. **Activate Part 2 config (`num_agent: 2`) and rebuild simulator:**
   ```bash
   cp ~/F1Tenth-Repository/src/gap_follow/config/sim_part2.yaml ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
   cd ~/F1Tenth-Repository && colcon build --packages-select f1tenth_gym_ros && source install/setup.bash
   ```
2. **Open 4 terminals and run in order:**
   * **Terminal 1 (Simulator):**
     ```bash
     cd ~/F1Tenth-Repository && source install/setup.bash
     ros2 launch f1tenth_gym_ros gym_bridge_launch.py
     ```
   * **Terminal 2 (Dual-Agent Rosbag Record):**
     ```bash
     cd ~/F1Tenth-Repository/bags && source ~/F1Tenth-Repository/install/setup.bash
     ros2 bag record -o rosbag_part2 /ego_racecar/odom /scan /drive /opp_racecar/odom /opp_scan /opp_drive
     ```
   * **Terminal 3 (Opponent Node — `5.5 m/s`):**
     ```bash
     cd ~/F1Tenth-Repository && source install/setup.bash
     ros2 run gap_follow reactive_node --ros-args -r __node:=opp_reactive_node -p scan_topic:=/opp_scan -p drive_topic:=/opp_drive -p odom_topic:=/opp_racecar/odom -p max_speed:=5.5 -p is_ego:=false
     ```
   * **Terminal 4 (Ego Node — `11.0 m/s` + Live Telemetry):**
     ```bash
     cd ~/F1Tenth-Repository && source install/setup.bash
     ros2 run gap_follow reactive_node
     ```
3. **Export Part 2 CSVs** (after pressing `Ctrl + C` in Terminals 4, 3, and 2):
   ```bash
   cd ~/F1Tenth-Repository/bags
   python3 ~/F1Tenth-Repository/src/gap_follow/scripts/bag_to_csv.py rosbag_part2 csv_part2
   ```
