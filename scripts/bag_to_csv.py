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

    # Ego CSV files
    ego_odom_f = open(os.path.join(out_dir, 'odometry.csv'), 'w', newline='')
    ego_lidar_f = open(os.path.join(out_dir, 'lidar.csv'), 'w', newline='')
    ego_vel_f = open(os.path.join(out_dir, 'velocity.csv'), 'w', newline='')

    ego_odom_w = csv.writer(ego_odom_f)
    ego_lidar_w = csv.writer(ego_lidar_f)
    ego_vel_w = csv.writer(ego_vel_f)

    odom_header = ['timestamp_s', 'pos_x_m', 'pos_y_m', 'yaw_rad', 'linear_vel_x_ms', 'angular_vel_z_rads']
    lidar_header = ['timestamp_s', 'min_dist_m', 'front_dist_m', 'right_dist_m', 'left_dist_m', 'max_dist_m']
    vel_header = ['timestamp_s', 'cmd_linear_vel_ms', 'cmd_steering_angle_rad', 'cmd_angular_vel_est_rads']

    ego_odom_w.writerow(odom_header)
    ego_lidar_w.writerow(lidar_header)
    ego_vel_w.writerow(vel_header)

    # Opponent CSV files (created automatically if /opp_* topics are present)
    if has_opp:
        opp_odom_f = open(os.path.join(out_dir, 'opp_odometry.csv'), 'w', newline='')
        opp_lidar_f = open(os.path.join(out_dir, 'opp_lidar.csv'), 'w', newline='')
        opp_vel_f = open(os.path.join(out_dir, 'opp_velocity.csv'), 'w', newline='')

        opp_odom_w = csv.writer(opp_odom_f)
        opp_lidar_w = csv.writer(opp_lidar_f)
        opp_vel_w = csv.writer(opp_vel_f)

        opp_odom_w.writerow(odom_header)
        opp_lidar_w.writerow(lidar_header)
        opp_vel_w.writerow(vel_header)

    wheelbase = 0.3302
    counts = {'ego_odom': 0, 'ego_lidar': 0, 'ego_vel': 0, 'opp_odom': 0, 'opp_lidar': 0, 'opp_vel': 0}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t_sec = t_ns * 1e-9
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == '/ego_racecar/odom':
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            ego_odom_w.writerow([
                f"{t_sec:.6f}", f"{msg.pose.pose.position.x:.4f}", f"{msg.pose.pose.position.y:.4f}",
                f"{yaw:.4f}", f"{msg.twist.twist.linear.x:.4f}", f"{msg.twist.twist.angular.z:.4f}"
            ])
            counts['ego_odom'] += 1

        elif topic == '/scan':
            min_d, front, right, left, max_d = extract_scan_metrics(msg.ranges)
            ego_lidar_w.writerow([
                f"{t_sec:.6f}", f"{min_d:.4f}", f"{front:.4f}", f"{right:.4f}", f"{left:.4f}", f"{max_d:.4f}"
            ])
            counts['ego_lidar'] += 1

        elif topic == '/drive':
            lin_v = msg.drive.speed
            steer = msg.drive.steering_angle
            ang_v = (lin_v / wheelbase) * math.tan(steer)
            ego_vel_w.writerow([f"{t_sec:.6f}", f"{lin_v:.4f}", f"{steer:.4f}", f"{ang_v:.4f}"])
            counts['ego_vel'] += 1

        elif has_opp and topic == '/opp_racecar/odom':
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            opp_odom_w.writerow([
                f"{t_sec:.6f}", f"{msg.pose.pose.position.x:.4f}", f"{msg.pose.pose.position.y:.4f}",
                f"{yaw:.4f}", f"{msg.twist.twist.linear.x:.4f}", f"{msg.twist.twist.angular.z:.4f}"
            ])
            counts['opp_odom'] += 1

        elif has_opp and topic == '/opp_scan':
            min_d, front, right, left, max_d = extract_scan_metrics(msg.ranges)
            opp_lidar_w.writerow([
                f"{t_sec:.6f}", f"{min_d:.4f}", f"{front:.4f}", f"{right:.4f}", f"{left:.4f}", f"{max_d:.4f}"
            ])
            counts['opp_lidar'] += 1

        elif has_opp and topic == '/opp_drive':
            lin_v = msg.drive.speed
            steer = msg.drive.steering_angle
            ang_v = (lin_v / wheelbase) * math.tan(steer)
            opp_vel_w.writerow([f"{t_sec:.6f}", f"{lin_v:.4f}", f"{steer:.4f}", f"{ang_v:.4f}"])
            counts['opp_vel'] += 1

    ego_odom_f.close()
    ego_lidar_f.close()
    ego_vel_f.close()
    if has_opp:
        opp_odom_f.close()
        opp_lidar_f.close()
        opp_vel_f.close()

    print(f"CSV export completed in: {out_dir}")
    print(f" - odometry.csv     ({counts['ego_odom']} samples)")
    print(f" - lidar.csv        ({counts['ego_lidar']} samples)")
    print(f" - velocity.csv     ({counts['ego_vel']} samples)")
    if has_opp:
        print(f" - opp_odometry.csv ({counts['opp_odom']} samples)")
        print(f" - opp_lidar.csv    ({counts['opp_lidar']} samples)")
        print(f" - opp_velocity.csv ({counts['opp_vel']} samples)")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 bag_to_csv.py <rosbag_folder_path> [output_csv_folder]")
        sys.exit(1)
    bag_folder = sys.argv[1]
    output_folder = sys.argv[2] if len(sys.argv) > 2 else bag_folder + "_csv"
    export_bag_to_csv(bag_folder, output_folder)
