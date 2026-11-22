# F1Tenth - Reactive Follow the Gap Controller

**Author:** Alain Sánchez | Mechatronics Engineering Student at ESPOL  
*Project developed for the Unmanned Vehicles course.*

<<<<<<< HEAD
This repository contains a high-performance Reactive Follow the Gap controller enhanced with a **Disparity Extender** and a **continuous velocity profile** for the F1Tenth ROS 2 simulator. The node is tuned for the Sakhir track, reaching top speeds of **11.0 m/s** and achieving sub-59-second lap times while maintaining stability across 10 consecutive laps.
=======
This repository contains a high-performance Reactive Follow the Gap controller enhanced with a **Disparity Extender** and a **continuous velocity profile** for the F1Tenth ROS 2 simulator (`f1tenth_gym_ros`). The project is structured into two progressive stages on the Sakhir circuit:
* **Part 1:** High-speed autonomous navigation (`11.0 m/s`, sub-59-second lap times) and stability across 10 consecutive collision-free laps.
* **Part 2:** Navigation through **5 static obstacles** on `Sakhir_obs_map` and **multi-agent dynamic overtaking**, where a second autonomous vehicle starts **5.0 m ahead** and is overtaken repeatedly across 10 consecutive laps without collisions.
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)

---

<<<<<<< HEAD
To run this controller on a custom track like Sakhir, configure the map files in your simulator workspace:

=======
## Repository Architecture & File Hierarchy

```text
PROYECTO-VNT/
├── config/
│   └── sim.yaml                                # Multi-agent (num_agent: 2) & Sakhir_obs_map configuration
├── gap_follow/
│   ├── __init__.py                             # Python package initializer
│   └── reactive_node.py                        # Main ROS 2 Reactive Follow the Gap + Disparity Extender node
├── maps/
│   ├── Sakhir_map.png                          # Original Sakhir track occupancy grid
│   ├── Sakhir_obs_map.png                      # Sakhir track map with 5 static obstacles inserted
│   └── Sakhir_obs_map.yaml                     # Map metadata (resolution, origin, occupied/free thresholds)
├── resource/
│   └── gap_follow                              # ROS 2 ament resource index marker
├── test/
│   ├── test_copyright.py                       # Code style and license tests
│   ├── test_flake8.py
│   └── test_pep257.py
├── Sakhir_map.png                              # Track layout preview for documentation
├── Screencast from 11-13-2026 09_01_10 PM.gif  # Part 1 simulation preview
├── package.xml                                 # ROS 2 package dependencies (rclpy, sensor_msgs, ackermann_msgs, etc.)
├── setup.cfg                                   # Installation script configuration
├── setup.py                                    # ROS 2 executable entry points (reactive_node)
└── README.md                                   # Complete project documentation (Part 1 & Part 2)
```

---

# Part 1: High-Speed Reactive Navigation & Telemetry HUD

### Video Demonstration (Part 1 - Solo Track & Static Obstacles)
* **YouTube Video (Part 1):** [Watch Part 1 Demo on YouTube](YOUR_YOUTUBE_LINK_PART_1)

## 1.1. Custom Map Setup (Sakhir Track)

To run this controller on a custom track like Sakhir, configure the map files in your simulator workspace:

>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
1. Extract the downloaded `.rar` or `.zip` archive containing the track data.
2. Locate the map files: a `.yaml` configuration file and a map image (`.png` or `.pgm`).
3. Move both files into your simulator's map directory (e.g., `src/f1tenth_gym_ros/maps/`).
4. Update the simulator configuration file (`sim.yaml` inside the `config` folder) or launch file so the `map_path` parameter points to your `sakhir` map.
5. Rebuild your workspace using `colcon build`.

![Sakhir Map Configuration](https://github.com/Frazz04/PROYECTO-VNT/blob/main/Sakhir_map.png)

## 1.2. Running the Simulator

Open a terminal, source your workspace, and launch the F1Tenth simulator:

```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

<<<<<<< HEAD
## 3. Running the Reactive Controller & RViz HUD
=======
## 1.3. Running the Reactive Controller & RViz HUD
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)

Once the simulator and RViz are open, launch the controller node in a second terminal:

```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 run gap_follow reactive_node
```

To visualize the live lap counter and stopwatch directly inside RViz, click **Add** in the **Displays** panel, select **Marker**, and set its topic to `/lap_telemetry_marker`.
<<<<<<< HEAD

## 4. Code Logic and Algorithm Explanation

Below is a step-by-step breakdown of the architecture implemented in `reactive_node.py`:

### 4.1. Node Initialization & Telemetry Setup
=======

## 1.4. Code Logic and Algorithm Explanation

Below is a step-by-step breakdown of the core architecture implemented in `reactive_node.py`:

### 1.4.1. Node Initialization & Parameterized Multi-Agent Setup
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
```python
class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
<<<<<<< HEAD
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/lap_telemetry_marker', 10)

        self.start_x = None
        self.start_y = None
        self.lap_count = 0
        self.lap_start_time = None
        self.last_lap_time = 0.0
        self.best_lap_time = float('inf')
        self.left_start_zone = False

        self.timer = self.create_timer(0.1, self.display_telemetry)
```
We initialize the ROS 2 node by subscribing to `/scan` for LiDAR readings and `/ego_racecar/odom` for vehicle position tracking. Publishers are created for `/drive` (Ackermann control commands) and `/lap_telemetry_marker` (3D telemetry HUD in RViz), alongside a 10 Hz timer to refresh lap metrics in real time.

### 4.2. Odometry-Based Lap Counter & Stopwatch
```python
    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = curr_x
            self.start_y = curr_y
            self.lap_start_time = time.time()
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
```
The initial spawn coordinates are recorded as the start/finish line. Once the vehicle travels more than `5.0 m` away from the start zone, crossing back within a `2.5 m` radius increments the lap counter, records the elapsed lap time, and updates the personal best record.

### 4.3. Disparity Extender
=======

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('opp_odom_topic', '/opp_racecar/odom')
        self.declare_parameter('max_speed', 11.0)
        self.declare_parameter('is_ego', True)
```
We initialize the ROS 2 node with configurable parameters for LiDAR, drive, odometry topics, and maximum velocity. This allows the exact same reactive algorithm to control both the primary vehicle (`ego_racecar`) and the secondary dynamic obstacle vehicle (`opp_racecar`).

### 1.4.2. Synchronized Odometry-Based Lap Counter & Stopwatch
```python
        # Start timer only once both vehicles begin moving (or ego alone in 1-agent mode)
        if not self.race_started:
            both_ready = self.ego_moving and (self.opp_moving if self.opp_seen else True)
            if both_ready:
                self.race_started = True
                self.lap_start_time = time.time()
            else:
                return
```
The initial spawn coordinates are recorded as the start/finish line. In multi-agent mode, the stopwatch remains at `0.00 s` until both vehicles move more than `0.15 m` from their starting grid positions. Once the vehicle travels more than `5.0 m` away from the start zone, crossing back within a `2.5 m` radius increments the lap counter, records the elapsed lap time, and updates the personal best record.

### 1.4.3. Disparity Extender
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
```python
    def apply_disparity_extender(self, ranges, angle_inc):
        orig = ranges.copy()
        extended = ranges.copy()
        n = len(orig)
        disparity_threshold = 0.5
<<<<<<< HEAD
        car_half_margin = 0.28
=======
        car_half_margin = 0.30
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)

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
<<<<<<< HEAD
Whenever adjacent LiDAR rays exhibit a sudden distance jump greater than `0.5 m` (indicating a corner apex or wall edge), the algorithm projects half the vehicle's width plus a safety margin (`0.28 m`) across the gap. This prevents the car from clipping inner corner walls during high-speed S-curves.

### 4.4. LiDAR Preprocessing & Frontal Cone
=======
Whenever adjacent LiDAR rays exhibit a sudden distance jump greater than `0.5 m` (indicating a corner apex, static obstacle edge, or opponent vehicle), the algorithm projects half the vehicle's width plus a safety margin (`0.30 m`) across the gap. This prevents the car from clipping inner corner walls or obstacles during high-speed S-curves and overtakes.

### 1.4.4. LiDAR Preprocessing & Frontal Cone
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
```python
    def preprocess_lidar(self, ranges, angle_min, angle_inc):
        proc_ranges = np.array(ranges)
        proc_ranges = np.nan_to_num(proc_ranges, nan=18.0, posinf=18.0, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0.0, 18.0)

        proc_ranges = self.apply_disparity_extender(proc_ranges, angle_inc)
<<<<<<< HEAD
        
        angles = angle_min + np.arange(len(proc_ranges)) * angle_inc
        blind_spots = np.where(np.abs(angles) > 1.30)[0]
        proc_ranges[blind_spots] = 0.0 
        
        return proc_ranges, angles
```
Raw scan values are sanitized and capped at a `18.0 m` lookahead horizon. After applying the disparity extension, angles beyond `±1.30` radians are zeroed out to restrict perception to the forward hemisphere.

### 4.5. Safety Bubble & Racing Line Target Selection
=======

        angles = angle_min + np.arange(len(proc_ranges)) * angle_inc
        blind_spots = np.where(np.abs(angles) > 1.30)[0]
        proc_ranges[blind_spots] = 0.0

        return proc_ranges, angles
```
Raw scan values are sanitized and capped at an `18.0 m` lookahead horizon. After applying the disparity extension, angles beyond `±1.30` radians are zeroed out to restrict perception to the forward hemisphere.

### 1.4.5. Safety Bubble & Racing Line Target Selection
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
```python
        center_cone = proc_ranges[np.abs(angles) < math.radians(1.5)]
        front_dist = float(np.min(center_cone)) if len(center_cone) > 0 else 18.0

        ranges = proc_ranges.copy()
<<<<<<< HEAD
        valid_ranges = np.where(ranges > 0.0, ranges, 18.0) 
        closest_idx = np.argmin(valid_ranges)

        bubble_radius = 55
=======
        valid_ranges = np.where(ranges > 0.0, ranges, 18.0)
        closest_idx = np.argmin(valid_ranges)

        bubble_radius = 42
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
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
        best_idx = int(0.35 * center_idx + 0.65 * deepest_idx)
```
<<<<<<< HEAD
A narrow `±1.5°` central ray (`front_dist`) monitors walls directly ahead without triggering false positives from side walls on straights. After placing a `55`-index safety bubble around the nearest obstacle and isolating the widest contiguous free gap, the target index (`best_idx`) blends `35%` of the gap's geometric center with `65%` of its deepest point (using the median index on ties to avoid directional bias), generating an aggressive yet smooth racing line.

### 4.6. Steering Deadband & Continuous Velocity Profile
```python
        target_distance = float(proc_ranges[best_idx])
        effective_dist = min(target_distance, front_dist)
=======
A narrow `±1.5°` central ray (`front_dist`) monitors walls directly ahead without triggering false positives from side walls on straights. After placing a `42`-index safety bubble around the nearest obstacle and isolating the widest contiguous free gap, the target index (`best_idx`) blends `35%` of the gap's geometric center with `65%` of its deepest point (using the median index on ties to avoid directional bias), generating an aggressive yet smooth racing line.

### 1.4.6. Steering Deadband, Overtaking Clearance & Continuous Velocity Profile
```python
        target_distance = float(proc_ranges[best_idx])
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
        angle = data.angle_min + (best_idx * data.angle_increment)

        if abs(angle) < math.radians(4):
            angle = 0.0

<<<<<<< HEAD
        min_speed = 3.4
        max_speed = 11.0

=======
        if abs(angle) < math.radians(5):
            effective_dist = min(target_distance, front_dist)
        else:
            effective_dist = target_distance

        min_speed = 3.5
        max_speed = self.max_speed

>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
        if effective_dist >= 10.5:
            v_dist = max_speed
        elif effective_dist <= 2.5:
            v_dist = min_speed
        else:
            ratio = (effective_dist - 2.5) / 8.0
            v_dist = min_speed + (max_speed - min_speed) * (ratio ** 1.3)

        if abs(angle) == 0.0:
            v_angle = max_speed
        else:
<<<<<<< HEAD
            v_angle = max(min_speed, 10.5 - 12.0 * abs(angle))

        velocity = min(v_dist, v_angle)
```
A `4°` steering deadband eliminates high-speed oscillation on straights. Instead of abrupt discrete speed steps, velocity is computed continuously between `3.4 m/s` and `11.0 m/s` by taking the minimum of a distance-based braking curve (`v_dist`) and a steering-angle limit (`v_angle`), ensuring smooth weight transfer into hairpins and progressive acceleration on corner exits.

### 4.7. Publishing the Drive Command
=======
            v_angle = max(min_speed, min(max_speed, 10.5 - 12.0 * abs(angle)))

        velocity = min(v_dist, v_angle)
```
A `4°` steering deadband eliminates high-speed oscillation on straights. When steering into an overtaking lane (`|angle| >= 5°`), `effective_dist` switches to `target_distance` (the open lane ahead past the obstacle/opponent), allowing the vehicle to maintain overtaking speed instead of braking behind the lead car. Velocity is computed continuously between `3.5 m/s` and `max_speed` by taking the minimum of `v_dist` and `v_angle`.

### 1.4.7. Publishing the Drive Command
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)
```python
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = float(velocity)
        self.publisher.publish(drive_msg)
```
<<<<<<< HEAD
The resulting steering angle and target speed are packaged into an `AckermannDriveStamped` message and published to `/drive`.
=======
The resulting steering angle and target speed are packaged into an `AckermannDriveStamped` message and published to `/drive` (or `/opp_drive`).
>>>>>>> 8747bab (Add Part 2: 5 static obstacles, multi-agent overtaking, and repo hierarchy)

![Node Execution and Track Performance](https://github.com/Frazz04/PROYECTO-VNT/blob/main/Screencast%20from%2011-13-2026%2009_01_10%20PM.gif)

---

# Part 2: 5 Static Obstacles & Multi-Agent Dynamic Overtaking

### Video Demonstration (Part 2 - 5 Obstacles + 2-Robot Overtaking)
* **YouTube Video (Part 2):** [Watch Part 2 Demo on YouTube](YOUR_YOUTUBE_LINK_PART_2)

## 2.1. Static Obstacles Map Generation (`Sakhir_obs_map`)

To evaluate obstacle avoidance across 10 consecutive laps, **5 static square obstacles** (`0.40 m × 0.40 m`) were embedded directly into the occupancy grid of the Sakhir track (`maps/Sakhir_obs_map.png` and `maps/Sakhir_obs_map.yaml`).
* Each obstacle is positioned inside the drivable track area at `0.65 m – 1.05 m` from a track wall and at least `14.0 m` away from the starting grid, preserving a clear passing lane of over `2.8 m` for both single-car avoidance and multi-agent overtaking.

## 2.2. Multi-Agent Simulator Configuration (`sim.yaml`)

In `f1tenth_gym_ros/config/sim.yaml`, two autonomous agents (`num_agent: 2`) are enabled on `Sakhir_obs_map`. The second vehicle (`opp_racecar`) is spawned **5.0 meters ahead** of the primary vehicle (`ego_racecar`) aligned with the main straight heading:

```yaml
    # map parameters
    map_path: '/home/alain/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sakhir_obs_map'
    map_img_ext: '.png'

    # opponent parameters
    num_agent: 2

    # ego starting pose on map
    sx: 0.0
    sy: 0.0
    stheta: -0.3547

    # opp starting pose on map (5.0 meters ahead along the main straight)
    sx1: 4.689
    sy1: -1.737
    stheta1: -0.3547
```

## 2.3. Running the Multi-Agent Overtaking Simulation

Both robots run the exact same reactive algorithm (`reactive_node.py`). The opponent vehicle (`opp_racecar`) operates at a capped speed (`max_speed:=5.5`) while navigating around the 5 static obstacles, acting as a dynamic obstacle that `ego_racecar` (`max_speed:=11.0`) detects via LiDAR and overtakes repeatedly across **10 consecutive collision-free laps**:

1. **Terminal 1 — Launch Simulator & RViz:**
   ```bash
   cd ~/F1Tenth-Repository
   source install/setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

2. **Terminal 2 — Launch Primary Vehicle (`ego_racecar`, `11.0 m/s`):**
   ```bash
   cd ~/F1Tenth-Repository
   source install/setup.bash
   ros2 run gap_follow reactive_node
   ```

3. **Terminal 3 — Launch Lead Vehicle (`opp_racecar`, 5 m ahead at `5.5 m/s`):**
   ```bash
   cd ~/F1Tenth-Repository
   source install/setup.bash
   ros2 run gap_follow reactive_node --ros-args -r __node:=opp_reactive_node -p scan_topic:=/opp_scan -p drive_topic:=/opp_drive -p odom_topic:=/opp_racecar/odom -p max_speed:=5.5 -p is_ego:=False
   ```
