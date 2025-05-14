import numpy as np 

from ..motion_models import velocity_motion_model, velocity_motion_model_2
from ..observation_models import odometry_observation_model, odometry_observation_model_2

class KalmanFilter:

    def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.02, 0.02, 0.01], obs_noise_std = [0.02, 0.02, 0.01]):
        self.mu = initial_state # Initial state estimate [x, y, theta]
        self.Sigma = initial_covariance # Initial uncertainty

        self.A, self.B = velocity_motion_model() # The action model to use. Returns A and B matrices

        # Standard deviations for the noise in x, y, and theta (process or action model noise)
        self.proc_noise_std = np.array(proc_noise_std)
        # Process noise covariance (R)
        self.R = np.diag(self.proc_noise_std ** 2)  # process noise covariance

        # Observation model (C)
        self.C = odometry_observation_model() # The observation model to use

        # Standard deviations for the noise in x, y, theta (observation or sensor model noise)
        self.obs_noise_std = np.array(obs_noise_std)
        # Observation noise covariance (Q)
        self.Q = np.diag(self.obs_noise_std ** 2)
            
    def predict(self, u, dt):
        # TODO: Implement Kalman filter prediction step
        # Predict the new mean (mu) using A, B, and control input u
        # Predict the new covariance (Sigma) using A and R
        # Predict state
        A = self.A()  
        B = self.B(self.mu, dt ) # Control input matrix B

        print(f"A shape: {A.shape}")
        print(f"B shape: {B.shape}")
        print(f"mu shape: {self.mu.shape}")
        print(f"u shape: {u.shape}")

        self.mu_pred = A @ self.mu + B @ u

        print(f"mu_pred shape: {self.mu_pred.shape}")

        # Predict covariance
        self.Sigma_pred = A @ self.Sigma @ A.T + self.R


        return self.mu_pred, self.Sigma_pred

    def update(self, mu_pred, Sigma_pred, z):
        # TODO: Implement Kalman filter correction step
        # Compute Kalman gain K
        # Update the mean (mu) with the measurement z
        # Update the covariance (Sigma)
        # Innovation (residual)
        C= self.C
        Q= self.Q

        print(f"C shape: {C.shape}")
        print(f"z shape: {z.shape}")
        print(f"mu_pred shape: {mu_pred.shape}")

        y = z - (C @ mu_pred)

        # Normalize angle difference (theta)
        y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))

        # Innovation covariance
        S = C @ Sigma_pred @ C.T + Q

        # Kalman gain
        K = Sigma_pred @ C.T @ np.linalg.inv(S)

        # Update state
        self.mu_update = mu_pred + K @ y

        print(f"K shape: {K.shape}")
        print(f"y shape: {y.shape}")
        print(f"mu_update shape: {self.mu_update.shape}")

        # Update covariance
        I = np.eye(self.Sigma.shape[0])
        self.Sigma_update = (I - K @ C) @ Sigma_pred
    

        return self.mu_update, self.Sigma_update

class KalmanFilter_2:
    def __init__(self, initial_state, initial_covariance,
                 proc_noise_std=[0.02]*6, obs_noise_std=[0.02]*6):

        self.mu = initial_state  # Initial state estimate [x, y, theta, vx, vy, omega]
        self.Sigma = initial_covariance  # Initial uncertainty

        self.A, self.B = velocity_motion_model_2()  # Motion model matrices

        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # Process noise covariance

        self.C = odometry_observation_model_2()  # Observation matrix
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Observation noise covariance

    def predict(self, u=None, dt=1.0):
        # TODO: Implement Kalman prediction step for full state (6D)
        # Pure KF: use only the A matrix to update the state and covariance
        pass

    def update(self, z):
        # TODO: Implement update step
        # Compute Kalman gain
        # Correct the predicted state with measurement
        # Update covariance
        pass
