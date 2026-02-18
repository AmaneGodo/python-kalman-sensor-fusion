import numpy as np

class Plant2D: 
    """
    Simulated 2D physical system (ground truth).

    State:
        x, y     - position
        vx, vy   - velocity

    The plant evolves using true acceleration and may include
    unknown disturbances that the estimator does not model.
    """

    def __init__(self, dt, x0, y0, vx0, vy0, ax_true, ay_true, x_disturbance, y_disturbance):
        self.dt = dt
        
        self.x = x0
        self.y = y0
        self.vx = vx0
        self.vy = vy0

        self.ax_true = ax_true
        self.ay_true = ay_true
        self.disturbance_x = x_disturbance
        self.disturbance_y = y_disturbance

    def step(self, ax_command, ay_command):
        """
        Propagate true state forward by dt.
        """
        self.ax_true = ax_command + self.disturbance_x
        self.ay_true = ay_command + self.disturbance_y

        # TODO: update velocity
        self.vx += self.ax_true * self.dt
        self.vy += self.ay_true * self.dt

        # TODO: update position
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt

    def get_state(self):
        return np.array([self.x, self.y, self.vx, self.vy])