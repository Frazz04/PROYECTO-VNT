# F1Tenth - Reactive Follow the Gap Controller

**Author:** Alain Sánchez | Mechatronics Engineering Student at ESPOL  
*Project developed for the Unmanned Vehicles course.*

This repository contains a high-performance Reactive Follow the Gap controller enhanced with a **Disparity Extender** and a **continuous velocity profile** for the F1Tenth ROS 2 simulator. The node is tuned for the Sakhir track, reaching top speeds of **11.0 m/s** and achieving sub-59-second lap times while maintaining stability across 10 consecutive laps.

## 1. Custom Map Setup (Sakhir Track)

To run this controller on a custom track like Sakhir, configure the map files in your simulator workspace:

1. Extract the downloaded `.rar` or `.zip` archive containing the track data.
2. Locate the map files: a `.yaml` configuration file and a map image (`.png` or `.pgm`).
3. Move both files into your simulator's map directory (e.g., `src/f1tenth_gym_ros/maps/`).
4. Update the simulator configuration file (`sim.yaml` inside the `config` folder) or launch file so the `map_path` parameter points to your `sakhir` map.
5. Rebuild your workspace using `colcon build`.

![Sakhir Map Configuration](https://github.com/Frazz04/PROYECTO-VNT/blob/main/Sakhir_map.png)

## 2. Running the Simulator

Open a terminal, source your workspace, and launch the F1Tenth simulator:

```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## 3. Running the Reactive Controller & RViz HUD

Once the simulator and RViz are open, launch the controller node in a second terminal:

```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 run gap_follow reactive_node
```

To visualize the live lap counter and stopwatch directly inside RViz, click **Add** in the **Displays** panel, select **Marker**, and set its topic to `/lap_telemetry_marker`.

## 4. Code Logic and Algorithm Explanation

Below is a step-by-step breakdown of the architecture implemented in `reactive_node.py`:

### 4.1. Node Initialization & Telemetry Setup
```python
class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
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
Whenever adjacent LiDAR rays exhibit a sudden distance jump greater than `0.5 m` (indicating a corner apex or wall edge), the algorithm projects half the vehicle's width plus a safety margin (`0.28 m`) across the gap. This prevents the car from clipping inner corner walls during high-speed S-curves.

### 4.4. LiDAR Preprocessing & Frontal Cone
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
Raw scan values are sanitized and capped at a `18.0 m` lookahead horizon. After applying the disparity extension, angles beyond `±1.30` radians are zeroed out to restrict perception to the forward hemisphere.

### 4.5. Safety Bubble & Racing Line Target Selection
```python
        center_cone = proc_ranges[np.abs(angles) < math.radians(1.5)]
        front_dist = float(np.min(center_cone)) if len(center_cone) > 0 else 18.0

        ranges = proc_ranges.copy()
        valid_ranges = np.where(ranges > 0.0, ranges, 18.0) 
        closest_idx = np.argmin(valid_ranges)

        bubble_radius = 55
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
A narrow `±1.5°` central ray (`front_dist`) monitors walls directly ahead without triggering false positives from side walls on straights. After placing a `55`-index safety bubble around the nearest obstacle and isolating the widest contiguous free gap, the target index (`best_idx`) blends `35%` of the gap's geometric center with `65%` of its deepest point (using the median index on ties to avoid directional bias), generating an aggressive yet smooth racing line.

### 4.6. Steering Deadband & Continuous Velocity Profile
```python
        target_distance = float(proc_ranges[best_idx])
        effective_dist = min(target_distance, front_dist)
        angle = data.angle_min + (best_idx * data.angle_increment)

        if abs(angle) < math.radians(4):
            angle = 0.0

        min_speed = 3.4
        max_speed = 11.0

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
            v_angle = max(min_speed, 10.5 - 12.0 * abs(angle))

        velocity = min(v_dist, v_angle)
```
A `4°` steering deadband eliminates high-speed oscillation on straights. Instead of abrupt discrete speed steps, velocity is computed continuously between `3.4 m/s` and `11.0 m/s` by taking the minimum of a distance-based braking curve (`v_dist`) and a steering-angle limit (`v_angle`), ensuring smooth weight transfer into hairpins and progressive acceleration on corner exits.

### 4.7. Publishing the Drive Command
```python
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = float(velocity)
        self.publisher.publish(drive_msg)
```
The resulting steering angle and target speed are packaged into an `AckermannDriveStamped` message and published to `/drive`.

![Node Execution and Track Performance](https://github.com/Frazz04/PROYECTO-VNT/blob/main/Screencast%20from%2011-13-2026%2009_01_10%20PM.gif)
