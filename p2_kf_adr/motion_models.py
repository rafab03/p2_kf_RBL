import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        return np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        

    def control_input_matrix_B(mu, delta_t):
        theta = mu[2]
        return np.array([
        [delta_t * np.cos(theta), 0],
        [delta_t * np.sin(theta), 0],
        [0, delta_t]
        ])
        

    return state_transition_matrix_A, control_input_matrix_B
def velocity_motion_model_2():
    def A():
        def devuelve(dt):
            return np.array([
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]
            ])
        return devuelve
        

    def B(mu, dt):
        return np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0]
        ])
        

    return A(), B
