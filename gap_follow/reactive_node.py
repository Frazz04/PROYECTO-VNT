import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def preprocess_lidar(self, ranges, angle_min, angle_inc):
        proc_ranges = np.array(ranges)
        # Visión a 18.0m: Equilibrio ideal entre anticipación y estabilidad
        proc_ranges = np.nan_to_num(proc_ranges, nan=18.0, posinf=18.0, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0.0, 18.0)
        
        # Cono frontal
        angles = angle_min + np.arange(len(proc_ranges)) * angle_inc
        blind_spots = np.where(np.abs(angles) > 1.30)[0]
        proc_ranges[blind_spots] = 0.0 
        
        return proc_ranges

    def lidar_callback(self, data):
        ranges = self.preprocess_lidar(data.ranges, data.angle_min, data.angle_increment)

        if np.all(ranges == 0.0):
            return

        valid_ranges = np.where(ranges > 0.0, ranges, 18.0) 
        closest_idx = np.argmin(valid_ranges)

        # Burbuja de seguridad intacta (la que te funciona bien)
        bubble_radius = 70
        start_idx = max(0, closest_idx - bubble_radius)
        end_idx = min(len(ranges), closest_idx + bubble_radius)
        ranges[start_idx:end_idx] = 0.0

        non_zeros = np.where(ranges > 0.0)[0]
        if len(non_zeros) == 0:
            return

        gaps = np.split(non_zeros, np.where(np.diff(non_zeros) != 1)[0] + 1)
        max_gap = max(gaps, key=len)

        # Buscar el objetivo
        center_idx = max_gap[len(max_gap) // 2]
        deepest_idx = max_gap[np.argmax(ranges[max_gap])]
        best_idx = int((center_idx + deepest_idx) / 2)

        target_distance = data.ranges[best_idx]
        angle = data.angle_min + (best_idx * data.angle_increment)

        # ---------------------------------------------------------
        # ZONA MUERTA para estabilizar rectas
        # ---------------------------------------------------------
        if abs(angle) < math.radians(4):
            angle = 0.0

        # ---------------------------------------------------------
        # VELOCIDADES: Ajuste de frenado anticipado para esquinas cerradas
        # ---------------------------------------------------------
        # Exigimos 12 metros libres (antes 10) para ir a 11 m/s. Así frena antes de la esquina.
        if target_distance > 12.0 and abs(angle) == 0.0:
            velocity = 11.0  
        # Bajamos a 7.5 m/s un poco antes (a los 8.5 metros) para quitar inercia
        elif target_distance > 8.5 and abs(angle) < math.radians(8):
            velocity = 7.5   
        # Entrada a curva más controlada
        elif target_distance > 5.5 and abs(angle) < math.radians(20):
            velocity = 5.5   
        elif abs(angle) < math.radians(35):
            velocity = 4.5   
        else:
            velocity = 3.0   

        # Publicar comando
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = float(velocity)
        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
