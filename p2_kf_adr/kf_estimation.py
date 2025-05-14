import numpy as np
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, Odom2DDriftSimulator, generate_noisy_measurement
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # TODO: Initialize filter with initial state and covariance
        initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 0.1
        self.last_time = None

        self.Visualizar = Visualizer()

        noise_config = self.declare_parameter('noise_config', 'ruido_bajo').value

        # Ajuste de desviaciones estándar según el tipo de configuración de ruido
        if noise_config == 'ruido_bajo':
            self.get_logger().info("Configuración de ruido: bajo")
            proc_noise_std = [0.02, 0.02, 0.01]   # ruido de proceso bajo
            self.obs_noise_std  = [0.02, 0.02, 0.01]   # ruido de medición bajo

        elif noise_config == 'ruido_alto_medida':
            self.get_logger().info("Configuración de ruido: alto en medida")
            proc_noise_std = [0.02, 0.02, 0.01]   # ruido de proceso bajo
            self.obs_noise_std  = [0.10, 0.10, 0.05]   # ruido de medición alto

        elif noise_config == 'ruido_alto_proceso':
            self.get_logger().info("Configuración de ruido: alto en proceso")
            proc_noise_std = [0.10, 0.10, 0.05]   # ruido de proceso alto
            self.obs_noise_std  = [0.02, 0.02, 0.01]   # ruido de medición bajo



        self.kf = KalmanFilter(initial_state, initial_covariance, proc_noise_std, self.obs_noise_std )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

    def odom_callback(self, msg):
        # Extract velocities and timestep
        pose= odom_to_pose2D(msg)
        v= msg.twist.twist.linear.x
        omega= msg.twist.twist.angular.z
        u=  np.array([v, omega])

        # Run predict() and update() of KalmanFilter
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = (current_time - self.last_time)
        self.last_time = current_time

        # Publish estimated state
        self.kf.predict(u, dt)

        ztotal= generate_noisy_measurement(pose, v, omega, None)
        z=  np.array([ztotal[0], ztotal[1], ztotal[2]])
        self.kf.update(z)

        # Publish the estimated state
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "odom"

        x, y, theta = self.kf.mu
        cov=self.kf.Sigma

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(theta / 2.0)
        quaternion.w = math.cos(theta / 2.0)
        pose_msg.pose.pose.orientation = quaternion

        # Set covariance
        pose_msg.pose.covariance = [
            # Row 0 (x)
            cov[0, 0], cov[0, 1], 0.0,    0.0,    0.0,    cov[0, 2],
            # Row 1 (y)
            cov[1, 0], cov[1, 1], 0.0,    0.0,    0.0,    cov[1, 2],
            # Row 2 (z) — no lo estimamos
            0.0,      0.0,      1e-6,    0.0,    0.0,    0.0,
            # Row 3 (roll) — no lo estimamos
            0.0,      0.0,      0.0,     1e-6,   0.0,    0.0,
            # Row 4 (pitch) — no lo estimamos
            0.0,      0.0,      0.0,     0.0,    1e-6,   0.0,
            # Row 5 (yaw)
            cov[2, 0], cov[2, 1], 0.0,    0.0,    0.0,    cov[2, 2],
        ]
        
        self.publisher.publish(pose_msg)

        # Update visualization
        pose_real= pose
        pose_estimada= self.kf.mu
        cov= self.kf.Sigma
        self.Visualizar.update(pose_real, pose_estimada, cov)
        



        pass

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()