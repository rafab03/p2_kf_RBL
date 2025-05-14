import numpy as np

def odometry_observation_model():
    return np.eye(3)  # Identity matrix (3x3) if all 3 state variables are observed
    

def odometry_observation_model_2():
    return np.eye(6)  # Return identity matrix (6x6) if all 6 state variables are observed COMPLETAR
    
