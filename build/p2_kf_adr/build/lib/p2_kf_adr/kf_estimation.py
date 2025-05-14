import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import PoseWithCovarianceStamped
from .sensor_utils import generate_noisy_measurement
from scipy.spatial.transform import Rotation as R
from tf_transformations import quaternion_from_euler


import numpy as np
import math
from time import time

from .sensor_utils import odom_to_pose2D,generate_noisy_measurement, get_normalized_pose2D, Odom2DDriftSimulator
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        
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
        # Initialize the Kalman Filter
        self.kf = KalmanFilter(np.zeros((3,1)), np.eye(3), proc_noise_std=[0.01, 0.01, 0.01], obs_noise_std=[0.1, 0.1, 0.1])

        self.get_logger().info(f'mu , sigma: {self.kf.mu}, {self.kf.Sigma}')

        # Initialize the drift simulator, to corrupt the perfect simulation odometry
        self.odom_simulator = Odom2DDriftSimulator()

        # Publishers for RViz visualization
        self.estimated_pose_publisher = self.create_publisher(PoseStamped, 'kf_estimated_pose', 10)
        self.estimated_path_publisher = self.create_publisher(Path, 'kf_estimated_path', 10)
        self.real_path_publisher = self.create_publisher(Path, 'real_path', 10)

        # Initialize the visualizer
        self.visualizer = Visualizer()

        self.u = np.zeros((2, 1))               # Initial controls [linear velocity, angular velocity]
        self.prev_time = None              # To compute time step (dt)
        self.first_prediction_done = False # Flag to track first prediction

        self.initial_pose = None           # Store initial pose for normalization
        self.normalized_pose = (0.0, 0.0, 0.0)

        # Paths for visualization
        self.estimated_path = Path()
        self.real_path = Path()

        self.last_time = None

    def odom_callback(self, msg):

        self.get_logger().info(f'recibido')
        self.get_logger().info(f'Mensaje completo: {msg}')

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.initial_pose is None:
            self.initial_pose = odom_to_pose2D(msg)
            self.prev_time = current_time
            return
        current_pose = odom_to_pose2D(msg)
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_pose, current_pose))

        x, y, yaw = self.normalized_pose
        self.get_logger().info(
            f'[DEBUG] Normalized Pose → x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f} rad'
        )


        #Time step (dt)
        
        if self.prev_time is None:
            self.prev_time = current_time
            return
        dt = current_time - self.prev_time
        self.prev_time = current_time

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular

        self.get_logger().info(
            f'[DEBUG] Linear Velocities → x: {linear.x:.2f}, y: {linear.y:.2f}, z: {linear.z:.2f}'
        )
        self.get_logger().info(
            f'[DEBUG] Angular Velocities → x: {angular.x:.2f}, y: {angular.y:.2f}, z: {angular.z:.2f}'
        )
        vlineal = math.sqrt(linear.x**2 + linear.y**2)

        z = generate_noisy_measurement(self.normalized_pose, vlineal, angular.z)


        
        
        self.get_logger().info(f"u: {self.u}")
        self.get_logger().info(f"dt: {dt}")

        mu_pred, Sigma_pred = self.kf.predict( self.u , dt)
        mu_update, Sigma_update = self.kf.update(mu_pred, Sigma_pred, z)

        self.get_logger().info(f"KF State: {mu_update}")
        self.get_logger().info(f"Noisy measurement: {z}")

        # Extraer valores escalares
        x_est = float(mu_update[0]) if mu_update.ndim == 1 else float(mu_update[0, 0])
        y_est = float(mu_update[1]) if mu_update.ndim == 1 else float(mu_update[1, 0])
        theta_est = float(mu_update[2]) if mu_update.ndim == 1 else float(mu_update[2, 0])


        # === Mensaje PoseStamped real ===
        real_pose_msg = PoseStamped()
        real_pose_msg.header.stamp = self.get_clock().now().to_msg()
        real_pose_msg.header.frame_id = "map"
        real_pose_msg.pose.position.x = x
        real_pose_msg.pose.position.y = y
        real_pose_msg.pose.position.z = 0.0
        q = R.from_euler('z', yaw).as_quat()
        real_pose_msg.pose.orientation.x = q[0]
        real_pose_msg.pose.orientation.y = q[1]
        real_pose_msg.pose.orientation.z = q[2]
        real_pose_msg.pose.orientation.w = q[3]

        # === Mensaje PoseStamped estimado ===
        est_pose_msg = PoseStamped()
        est_pose_msg.header = real_pose_msg.header
        est_pose_msg.pose.position.x = x_est
        est_pose_msg.pose.position.y = y_est
        est_pose_msg.pose.position.z = 0.0
        q2 = R.from_euler('z', theta_est).as_quat()
        est_pose_msg.pose.orientation.x = q2[0]
        est_pose_msg.pose.orientation.y = q2[1]
        est_pose_msg.pose.orientation.z = q2[2]
        est_pose_msg.pose.orientation.w = q2[3]

        # === Publicar paths en RViz ===
        self.real_path.header = real_pose_msg.header
        self.real_path.poses.append(real_pose_msg)
        self.real_path_publisher.publish(self.real_path)

        self.estimated_path.header = est_pose_msg.header
        self.estimated_path.poses.append(est_pose_msg)
        self.estimated_path_publisher.publish(self.estimated_path)


    

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()