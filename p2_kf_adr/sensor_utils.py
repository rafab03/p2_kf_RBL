import numpy as np
import PyKDL

def odom_to_pose2D(odom):
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    yaw = get_yaw_from_quaternion(odom.pose.pose.orientation)
    return (x, y, yaw)

def get_yaw_from_quaternion(quaternion):
    rot = PyKDL.Rotation.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    return rot.GetRPY()[2]
def get_normalized_pose2D(initial_pose, current_pose):
    # Check if the initial pose is set
    if initial_pose:
        x, y, yaw = current_pose
        init_x, init_y, init_yaw = initial_pose

        # Adjust position
        x -= init_x
        y -= init_y

        # Adjust orientation
        yaw -= init_yaw
        yaw = normalize_angle(yaw)

        return (x, y, yaw)
    else:
        return (0.0, 0.0, 0.0)  # Default pose if initial pose not set
def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

class Odom2DDriftSimulator:
    def __init__(self):
        self.error_accumulation = np.array([0.0, 0.0, 0.0])  # Accumulative drift error
        self.last_update = None

    def add_drift(self, odom, current_time):
        if self.last_update is None:
            self.last_update = current_time
            return odom

        time_delta = current_time - self.last_update
        self.last_update = current_time

        # Increase error over time or based on some condition
        drift_rate = np.array([0.001, 0.001, 0.0001])  # Adjust these rates as needed
        self.error_accumulation += drift_rate * time_delta
        print("Error",self.error_accumulation)

        # Optional: Add random walk
        random_walk = np.random.normal(0, [0.001, 0.001, 0.0001], 3)

        # Apply the drift to odometry
        drifted_odom = odom + self.error_accumulation + random_walk

        return drifted_odom

def generate_noisy_measurement(pose, v, omega, noise_std=None):
    """
    Adds Gaussian noise to the observation [x, y, theta, v, omega].

    :param pose: Tuple (x, y, theta) - position and orientation
    :param v: Linear velocity (float)
    :param omega: Angular velocity (float)
    :param noise_std: Optional list/array of standard deviations for each element
    :return: Noisy observation as a numpy array
    """
    if noise_std is None:
        noise_std = np.array([0.02, 0.02, 0.01, 0.02, 0.02])
    
    noise = np.random.normal(0, noise_std)
    z = np.array([pose[0], pose[1], pose[2], v, omega]) + noise
    return z
def generate_noisy_measurement_2(pose, vx, vy, omega, noise_std=None):
    """
    Adds Gaussian noise to the observation [x, y, theta, vx, vy, omega].

    :param pose: Tuple (x, y, theta) - position and orientation
    :param v: Linear velocity (float)
    :param omega: Angular velocity (float)
    :param noise_std: Optional list/array of standard deviations for each element
    :return: Noisy observation as a numpy array
    """
    if noise_std is None:
        noise_std = np.array([0.02, 0.02, 0.01, 0.02, 0.02, 0.01])
    

    x, y, theta = pose
    true_measurement = np.array([x, y, theta, vx, vy, omega])
    noise = np.random.normal(0, noise_std, size=6)
    return (true_measurement + noise)