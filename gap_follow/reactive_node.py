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
