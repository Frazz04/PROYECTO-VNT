import sys
import os
import csv
import math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


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

    odom_file = open(os.path.join(out_dir, 'odometry.csv'), 'w', newline='')
    lidar_file = open(os.path.join(out_dir, 'lidar.csv'), 'w', newline='')
    vel_file = open(os.path.join(out_dir, 'velocity.csv'), 'w', newline='')

    odom_writer = csv.writer(odom_file)
    lidar_writer = csv.writer(lidar_file)
    vel_writer = csv.writer(vel_file)

    odom_writer.writerow(['timestamp_s', 'pos_x_m', 'pos_y_m', 'yaw_rad', 'linear_vel_x_ms', 'angular_vel_z_rads'])
    lidar_writer.writerow(['timestamp_s', 'min_dist_m', 'front_dist_m', 'right_dist_m', 'left_dist_m', 'max_dist_m'])
    vel_writer.writerow(['timestamp_s', 'cmd_linear_vel_ms', 'cmd_steering_angle_rad', 'cmd_angular_vel_est_rads'])

    wheelbase = 0.3302
    counts = {'odom': 0, 'lidar': 0, 'vel': 0}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t_sec = t_ns * 1e-9
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == '/ego_racecar/odom':
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            odom_writer.writerow([
                f"{t_sec:.6f}",
                f"{msg.pose.pose.position.x:.4f}",
                f"{msg.pose.pose.position.y:.4f}",
                f"{yaw:.4f}",
                f"{msg.twist.twist.linear.x:.4f}",
                f"{msg.twist.twist.angular.z:.4f}"
            ])
            counts['odom'] += 1

        elif topic == '/scan':
            valid = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > 0.0]
            n = len(msg.ranges)
            front = msg.ranges[n // 2] if n > 0 else 0.0
            right = msg.ranges[int(n * 0.25)] if n > 0 else 0.0
            left = msg.ranges[int(n * 0.75)] if n > 0 else 0.0
            min_d = min(valid) if valid else 0.0
            max_d = max(valid) if valid else 0.0
            lidar_writer.writerow([
                f"{t_sec:.6f}",
                f"{min_d:.4f}",
                f"{front:.4f}",
                f"{right:.4f}",
                f"{left:.4f}",
                f"{max_d:.4f}"
            ])
            counts['lidar'] += 1

        elif topic == '/drive':
            lin_v = msg.drive.speed
            steer = msg.drive.steering_angle
            ang_v = (lin_v / wheelbase) * math.tan(steer)
            vel_writer.writerow([
                f"{t_sec:.6f}",
                f"{lin_v:.4f}",
                f"{steer:.4f}",
                f"{ang_v:.4f}"
            ])
            counts['vel'] += 1

    odom_file.close()
    lidar_file.close()
    vel_file.close()

    print(f"CSV export completed in: {out_dir}")
    print(f" - odometry.csv ({counts['odom']} samples)")
    print(f" - lidar.csv    ({counts['lidar']} samples)")
    print(f" - velocity.csv ({counts['vel']} samples)")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 bag_to_csv.py <rosbag_folder_path> [output_csv_folder]")
        sys.exit(1)
    bag_folder = sys.argv[1]
    output_folder = sys.argv[2] if len(sys.argv) > 2 else bag_folder + "_csv"
    export_bag_to_csv(bag_folder, output_folder)
