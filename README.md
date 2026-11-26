# F1Tenth Autonomous Navigation: Reactive Obstacle Avoidance & Multi-Agent Overtaking

## Project Presentation

| Field | Details |
| :--- | :--- |
| **Institution** | Escuela Superior Politécnica del Litoral (ESPOL) |
| **Program / Area** | Mechatronics / Control & Autonomous Robotics |
| **Author** | Alain |
| **Simulation Environment** | ROS 2 Humble — `f1tenth_gym_ros` (`Sakhir_obs_map`) |
| **Control Strategy** | Purely Reactive Follow the Gap (FTG) + Disparity Extender |

### Quick Navigation
* [Repository File Hierarchy](#repository-file-hierarchy)
* [PART 1: Single-Agent Navigation with Static Obstacles](#part-1-single-agent-navigation-with-static-obstacles)
* [PART 2: Multi-Agent Dynamic Overtaking & Dual Telemetry](#part-2-multi-agent-dynamic-overtaking--dual-telemetry)

---

## Repository File Hierarchy

```text
gap_follow/
├── config/
│   ├── sim.yaml                 # Active simulator configuration
│   ├── sim_part1.yaml           # Part 1 configuration (Sakhir_obs_map, num_agent: 1)
│   └── sim_part2.yaml           # Part 2 configuration (Sakhir_obs_map, num_agent: 2, sx1: 4.689, sy1: -1.737)
├── gap_follow/
│   ├── __init__.py              # Python package initializer
│   └── reactive_node.py         # Parameterized ROS 2 reactive navigation & telemetry node
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
---

# PART 1: Single-Agent Navigation with Static Obstacles

## 1.1 Media & Telemetry Datasets (Part 1)

* **YouTube Demonstration (Part 1):** [Watch Part 1 Video on YouTube](https://www.youtube.com/watch?v=FEg4f36rbRs)
* **Google Drive Repository (Part 1 Rosbag & CSV Files):** [Download Part 1 Rosbag & CSV Data](https://drive.google.com/drive/folders/1SRFstlYe2fw5lLFCmOm4M3tun3Aivstm?usp=sharing)
  * *Included files:* `rosbag_part1/` (`.db3` + `metadata.yaml`) and converted CSV tables (`odometry.csv`, `lidar.csv`, `velocity.csv`).

---

## 1.2 Introduction & Objectives (Part 1)

Autonomous racing at high speeds requires real-time perception and decision-making capable of reacting to unexpected obstacles without losing stability. In **Part 1**, a single 1/10th-scale Ackermann racecar (`ego_racecar`, up to `11.0 m/s`) is evaluated in **ROS 2 Humble** inside `f1tenth_gym_ros` on the **Sakhir** circuit modified with **5 static obstacles** (`Sakhir_obs_map`) placed along key straights and braking zones.

### Objectives
1. **Mapless Reactive Navigation:** Navigate `Sakhir_obs_map` cleanly using solely instantaneous 2D LiDAR scans (`/scan`), without reading the occupancy map file or using global trajectory planners.
2. **Collision-Free Static Obstacle Evasion:** Prevent corner-clipping collisions against the 5 static blocks (`0.40 m x 0.40 m`) and hairpin walls by combining a **Disparity Extender**, a **Safety Bubble**, and adaptive gap centering.
3. **Real-Time Telemetry & Rosbag-to-CSV Pipeline:** Display live lap timing, odometry (`X`, `Y`, `Yaw`, linear and angular velocities), LiDAR clearances, and velocity commands in the terminal at `4 Hz`, while recording `/ego_racecar/odom`, `/scan`, and `/drive` via `ros2 bag record` and converting them to `.csv` files.

---

## 1.3 Methodology (Part 1)

1. **LiDAR Preprocessing & Lookahead Clipping:** Raw ranges are sanitized (`NaN`/`Inf`) and clipped to a maximum lookahead horizon of **`18.0 m`** to avoid excessive speed buildup on long straights, then masked outside $[-1.30, +1.30]\text{ rad}$ ($\pm 74.5^\circ$) to focus on the forward hemisphere.
2. **Disparity Extender:** Scans adjacent rays for abrupt depth jumps ($\vert{}\Delta r\vert{} > 0.5\text{ m}$) and expands the closer obstacle distance across an angular arc $\theta_{\text{ext}} = \arctan(0.28 / r_{\text{close}})$ to account for half the vehicle's physical width plus safety clearance.
3. **Safety Bubble & Adaptive Gap Selection:** Zeroes out a `44`-index safety bubble around the nearest obstacle (`closest_idx`), finds the widest contiguous corridor (`max_gap`), and computes the target ray by blending `65% center + 35% deepest` in narrow obstacle corridors (`< 160` rays) or `35% center + 65% deepest` in open corners (`>= 160` rays).
4. **Steering Deadband & Multi-Constraint Speed Control:** Snaps steering angles smaller than `±4.0°` to `0.0 rad` to prevent high-speed straightaway oscillations, and sets linear speed (`3.4 m/s` to `11.0 m/s`) as the minimum of three continuous limits: forward clearance (`v_dist`), steering angle (`v_angle`), and lateral proximity (`v_prox`).

---

## 1.4 Code Walkthrough (Part 1)

### A. Disparity Extender & LiDAR Preprocessing (`reactive_node.py`)

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

* **What this block does:** Caps sensor lookahead at `18.0 m`, inflates every sharp edge ($\vert{}\Delta r\vert{} > 0.5\text{ m}$) by a `0.28 m` lateral safety margin so the wheels clear all 5 static obstacles, and filters out rear side walls beyond $\pm 1.30\text{ rad}$.

### B. Safety Bubble, Adaptive Target Aiming & Speed Control (`reactive_node.py`)

```python
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
        gaps = np.split(non_zeros, np.where(np.diff(non_zeros) != 1)[0] + 1)
        max_gap = max(gaps, key=len)

        center_idx = max_gap[len(max_gap) // 2]
        max_depth = np.max(ranges[max_gap])
        deepest_candidates = np.where(ranges[max_gap] == max_depth)[0]
        deepest_idx = max_gap[deepest_candidates[len(deepest_candidates) // 2]]

        if len(max_gap) < 160:
            best_idx = int(0.65 * center_idx + 0.35 * deepest_idx)
        else:
            best_idx = int(0.35 * center_idx + 0.65 * deepest_idx)
```

* **What this block does:** Creates a `44`-ray exclusion zone around the closest obstacle, finds the widest free corridor `max_gap`, and shifts aiming weight toward the corridor's geometric center (`65%`) when passing narrow obstacle gaps.

### C. Single-Agent Telemetry & CSV Export (`display_telemetry` & `bag_to_csv.py`)

```python
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
```

* **What this block does:** Prints real-time `[LAPS]`, `[ODOMETRY]`, `[LIDAR]`, and `[VELOCITY]` diagnostics in English at `4 Hz` while `ros2 bag record` logs `/ego_racecar/odom`, `/scan`, and `/drive` for conversion into `odometry.csv`, `lidar.csv`, and `velocity.csv`.

---

## 1.5 Build, Execution & CSV Export Workflow (Part 1)

1. **Activate Part 1 configuration (`num_agent: 1`) and build:**
   ```bash
   cp ~/F1Tenth-Repository/src/gap_follow/config/sim_part1.yaml ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
   cd ~/F1Tenth-Repository
   colcon build --packages-select f1tenth_gym_ros gap_follow
   source install/setup.bash
   ```

2. **Open 3 terminals and run in order:**
   * **Terminal 1 — Launch Simulator (`Sakhir_obs_map`, 1 agent):**
     ```bash
     cd ~/F1Tenth-Repository && source install/setup.bash
     ros2 launch f1tenth_gym_ros gym_bridge_launch.py
     ```
   * **Terminal 2 — Record Part 1 Rosbag (`Odometry`, `LiDAR`, `Velocity`):**
     ```bash
     mkdir -p ~/F1Tenth-Repository/bags && cd ~/F1Tenth-Repository/bags
     source ~/F1Tenth-Repository/install/setup.bash
     ros2 bag record -o rosbag_part1 /ego_racecar/odom /scan /drive
     ```
   * **Terminal 3 — Launch Ego Reactive Controller:**
     ```bash
     cd ~/F1Tenth-Repository && source install/setup.bash
     ros2 run gap_follow reactive_node
     ```

3. **Convert Rosbag to CSV** (after stopping Terminal 3 and Terminal 2 with `Ctrl + C`):
   ```bash
   cd ~/F1Tenth-Repository/bags
   python3 ~/F1Tenth-Repository/src/gap_follow/scripts/bag_to_csv.py rosbag_part1 csv_part1
   ```
   *Output files in `csv_part1/`:* `odometry.csv`, `lidar.csv`, `velocity.csv`.

---
---

# PART 2: Multi-Agent Dynamic Overtaking & Dual Telemetry

## 2.1 Media & Telemetry Datasets (Part 2)

* **YouTube Demonstration (Part 2):** [Watch Part 2 Video on YouTube](https://www.youtube.com/watch?v=VInKZxv27xs)
* **Google Drive Repository (Part 2 Rosbag & CSV Files):** [Download Part 2 Rosbag & CSV Data](https://drive.google.com/drive/folders/1iEBmhceOEppFtZkpx0G388o3cwOAI9Wc?usp=sharing)
  * *Included files:* `rosbag_part2/` (`.db3` + `metadata.yaml`) and converted CSV tables for both vehicles (`odometry.csv`, `lidar.csv`, `velocity.csv`, `opp_odometry.csv`, `opp_lidar.csv`, `opp_velocity.csv`).

---

## 2.2 Introduction & Objectives (Part 2)

In **Part 2**, the simulation environment is upgraded to a **multi-agent racing scenario** (`num_agent: 2`) on `Sakhir_obs_map`. Two independent autonomous vehicles run simultaneously on the circuit:
* **Ego Racecar (`ego_racecar`):** Controlled by `reactive_node` with a maximum speed of **`11.0 m/s`**, starting at `(0.0, 0.0)`.
* **Opponent Racecar (`opp_racecar`):** Controlled by `opp_reactive_node` with a maximum speed of **`5.5 m/s`**, spawned **`5.0 m` ahead** on the main straight (`sx1: 4.689, sy1: -1.737, stheta1: -0.3547 rad`).

### Objectives
1. **Parameterized Dual-Node Execution:** Control both vehicles simultaneously from a single parameterized `reactive_node.py` executable by remapping node names, sensor/actuator topics, and speed limits at runtime.
2. **High-Speed Dynamic Overtaking:** Detect the slower lead vehicle via LiDAR, steer into an open adjacent lane with safe lateral clearance (`Disparity Extender`), and suppress premature frontal braking during the lane change to overtake cleanly without collisions.
3. **Synchronized Multi-Agent Stopwatch & Dual Rosbag/CSV Logging:** Synchronize the race timer so it starts at `0.00 s` only once **both** vehicles begin moving, while recording all 6 topics (`/ego_racecar/odom`, `/scan`, `/drive`, `/opp_racecar/odom`, `/opp_scan`, `/opp_drive`) and exporting separate CSV datasets for both agents.

---

## 2.3 Methodology & Multi-Agent Architecture (Part 2)

| Parameter / Interface | Ego Node (`reactive_node`) | Opponent Node (`opp_reactive_node`) | Role in Multi-Agent Simulation |
| :--- | :--- | :--- | :--- |
| **Node Name (`__node`)** | `reactive_node` | `opp_reactive_node` | Isolates both controllers in the ROS 2 computation graph |
| **`scan_topic`** | `/scan` | `/opp_scan` | Independent 2D LiDAR sensor streams (`LaserScan`) |
| **`drive_topic`** | `/drive` | `/opp_drive` | Independent Ackermann steering & speed commands |
| **`odom_topic`** | `/ego_racecar/odom` | `/opp_racecar/odom` | Independent pose & velocity feedback (`Odometry`) |
| **`opp_odom_topic`** | `/opp_racecar/odom` | `/opp_racecar/odom` | Monitored by Ego to synchronize stopwatch start |
| **`max_speed`** | `11.0 m/s` | `5.5 m/s` | Speed differential enables Ego to catch and pass Opp |
| **`is_ego`** | `True` | `False` | Enables `4 Hz` terminal telemetry & RViz2 HUD only on Ego |

1. **Synchronized Multi-Terminal Start:** Because two separate terminals launch `opp_reactive_node` and `reactive_node`, `odom_callback` holds the stopwatch at `0.00 s` until both `ego_racecar` and `opp_racecar` have moved more than `0.15 m` from their starting grids.
2. **Overtaking Velocity Override (`is_overtaking`):** Normally, `effective_dist = min(target_distance, front_dist)` forces braking whenever an obstacle enters the frontal cone. During an overtake, `opp_racecar` temporarily occupies `front_dist` (`< 6.5 m`) while the adjacent lane (`target_distance`) is clear (`> front_dist + 2.5 m`). The controller detects this geometry and maintains up to `7.5 m/s` to complete the pass smoothly.
3. **Dual-Agent Rosbag Deserialization:** `scripts/bag_to_csv.py` automatically detects `/opp_*` topics inside `rosbag_part2` and generates 6 synchronized CSV files (3 for Ego and 3 for Opponent).

---

## 2.4 Code Walkthrough (Part 2)

### A. Parameterized Dual-Node Initialization (`reactive_node.py`)

```python
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
```

* **What this block does:** Allows launching `opp_reactive_node` on `/opp_scan`, `/opp_drive`, and `/opp_racecar/odom` at `5.5 m/s` (`is_ego:=false`) while `reactive_node` runs on `/scan`, `/drive`, and `/ego_racecar/odom` at `11.0 m/s` (`is_ego:=true`).

### B. Synchronized Dual-Agent Start (`opp_odom_callback` & `odom_callback`)

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
        # ... pose & velocity extraction ...
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
```

* **What this block does:** Tracks initial displacement (`> 0.15 m`) for both `ego_racecar` and `opp_racecar` so the lap stopwatch starts strictly when both cars are actively racing.

### C. Dynamic Overtaking Detection & Speed Override (`lidar_callback`)

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
```

* **What this block does:** Detects when `ego_racecar` closes within `6.5 m` of `opp_racecar` and steers toward a free side corridor (`target_distance > front_dist + 2.5 m`), maintaining `~7.5 m/s` to pass the `5.5 m/s` opponent without slamming the brakes behind it.

### D. Dual-Agent CSV Export (`scripts/bag_to_csv.py`)

```python
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    has_opp = any(t.startswith('/opp_') for t in type_map)
```

* **What this block does:** Automatically detects `/opp_racecar/odom`, `/opp_scan`, and `/opp_drive` in `rosbag_part2` and writes `opp_odometry.csv`, `opp_lidar.csv`, and `opp_velocity.csv` alongside the Ego CSV tables.

---

## 2.5 Build, Execution & Dual-CSV Export Workflow (Part 2)

1. **Activate Part 2 configuration (`num_agent: 2`) and build:**
   ```bash
   cp ~/F1Tenth-Repository/src/gap_follow/config/sim_part2.yaml ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
   cd ~/F1Tenth-Repository
   colcon build --packages-select f1tenth_gym_ros gap_follow
   source install/setup.bash
   ```

2. **Open 4 terminals and run in order:**
   * **Terminal 1 — Launch Simulator (`Sakhir_obs_map`, 2 agents):**
     ```bash
     cd ~/F1Tenth-Repository && source install/setup.bash
     ros2 launch f1tenth_gym_ros gym_bridge_launch.py
     ```
   * **Terminal 2 — Record Dual-Agent Rosbag (Ego + Opponent topics):**
     ```bash
     cd ~/F1Tenth-Repository/bags && source ~/F1Tenth-Repository/install/setup.bash
     ros2 bag record -o rosbag_part2 /ego_racecar/odom /scan /drive /opp_racecar/odom /opp_scan /opp_drive
     ```
   * **Terminal 3 — Launch Opponent Node (`opp_racecar` at `5.5 m/s`):**
     ```bash
     cd ~/F1Tenth-Repository && source install/setup.bash
     ros2 run gap_follow reactive_node --ros-args -r __node:=opp_reactive_node -p scan_topic:=/opp_scan -p drive_topic:=/opp_drive -p odom_topic:=/opp_racecar/odom -p max_speed:=5.5 -p is_ego:=false
     ```
   * **Terminal 4 — Launch Ego Node (`ego_racecar` at `11.0 m/s` + Live Telemetry):**
     ```bash
     cd ~/F1Tenth-Repository && source install/setup.bash
     ros2 run gap_follow reactive_node
     ```

3. **Convert Rosbag to CSV** (after stopping Terminals 4, 3, and 2 with `Ctrl + C`):
   ```bash
   cd ~/F1Tenth-Repository/bags
   python3 ~/F1Tenth-Repository/src/gap_follow/scripts/bag_to_csv.py rosbag_part2 csv_part2
   ```
   *Output files in `csv_part2/`:* `odometry.csv`, `lidar.csv`, `velocity.csv`, `opp_odometry.csv`, `opp_lidar.csv`, `opp_velocity.csv`.
