import numpy as np

class KalmanFilter2D:
    """
    State: [x, y, vx, vy, bax, bay]

    Fuses IMU acceleration and GPS position.
    """

    def __init__(self, dt, x0, y0, vx0, vy0, bax, bay, Q_imu, Q_bias, R_gps):
        self.dt = dt

        # state estimate
        self.x_hat = [x0, y0, vx0, vy0, bax, bay]

        # Uncertainty for prectice
        # positional uncertainty
        self.P_x = 1.0
        self.P_y = 0.8

        # velocity uncertainty
        self.P_vx = 1.0
        self.P_vy = 0.75
        
        # bias (acceleration) uncertainty
        self.P_bx = 0.1
        self.P_by = 0.08

        # Q: how noisy the IMU acceleration is 
        self.Q_imu = Q_imu
        self.Q_bias = Q_bias

        self.R_gps = R_gps

        # bias terms
        self.bias_alpha = 0.001
        self.bias_clip = 0.5

    def predict(self, ax_meas, ay_meas):
        """
        Propagate state using IMU.
        """
        # IMU measure ax_measured and ay_measured
        ax_used = ax_meas - self.x_hat[4]
        ay_used = ay_meas - self.x_hat[5]

        # pos_predict (both x and y) = pos_0 + (v * dt) + (a_measured * dt^2)/2
        self.x_hat[0] = self.x_hat[0] + self.x_hat[2] * self.dt + (ax_used * self.dt**2)/2
        self.x_hat[1] = self.x_hat[1] + self.x_hat[3] * self.dt + (ay_used * self.dt**2)/2

        # v_predict (both x and y) = v + a_measured * dt
        self.x_hat[2] = self.x_hat[2] + ax_used * self.dt
        self.x_hat[3] = self.x_hat[3] + ay_meas * self.dt
        
        # self.x_hat[1] = self.x_hat[1] + self.x_hat[3] * self.dt + (ay_meas * self.dt**2)/2

        # P: uncertainty
        # IMU input -> a_measured: biased and cause drift
            # if a_measured is uncertain -> velocity = v + a_meas * dt thus velocity becomes uncertain
            # if velocity is uncertain   -> position = x + (v * dt) + (a_meas * dt**2)/2 thus position also become uncertain

        # IMU uncertainty = combination of noise from IMU and acceleration bias uncertainty
        Q_eff_x = self.Q_imu + self.P_bx
        Q_eff_y = self.Q_imu + self.P_by

        # uncertainty: 
        # Acceleration Unceratainty: previous uncertainty + bias uncertainty * dt
        P_bx_new = self.P_bx + self.Q_bias * self.dt
        P_by_new = self.P_by + self.Q_bias * self.dt

        # Velocity Unceratainty: previous velocity uncertainty and uncertainty caused by acceleration bias
        P_vx_new = self.P_vx + Q_eff_x * self.dt**2 
        P_vy_new = self.P_vy + Q_eff_y * self.dt**2

        # Position Unceratainty: previous uncertainty + velocity uncertainty caused by acceleration unceratainty + acceleration bias
        P_x_new = self.P_x + self.P_vx * self.dt**2 + (Q_eff_x * self.dt**4)/4
        P_y_new = self.P_y + self.P_vy * self.dt**2 + (Q_eff_y * self.dt**4)/4
        
        # update uncertainty
        self.P_bx = P_bx_new
        self.P_by = P_by_new

        self.P_vx = P_vx_new
        self.P_vy = P_vy_new

        self.P_y = P_y_new
        self.P_x = P_x_new


    def update_gps(self, z_gps_x, z_gps_y):
        """
        GPS correction step.
        """

        # Compute innvation: innovation = z_gps - x_prediction
        innovation_x = z_gps_x - self.x_hat[0]
        innovation_y = z_gps_y - self.x_hat[1]

        # Compute Kalman gain: K = predicted position uncertainty / (predicted position uncertainty + GPS uncertainty)
        K_x = self.P_x / (self.P_x + self.R_gps)
        K_y = self.P_y / (self.P_y + self.R_gps)

        # Correct state estimate: 
        # estimated position = predicted position + K * innovation
        self.x_hat[0] = self.x_hat[0] + K_x * innovation_x
        self.x_hat[1] = self.x_hat[1] + K_y * innovation_y

        # esimated acceleration bias = current acceleration bias - K_bias * innovation
        bias_correction_x = np.clip(innovation_x, -self.bias_clip, self.bias_clip)
        self.x_hat[4] = self.x_hat[4] + self.bias_alpha * bias_correction_x

        bias_correction_y = np.clip(innovation_y, -self.bias_clip, self.bias_clip)
        self.x_hat[5] = self.x_hat[5] + self.bias_alpha * bias_correction_y

        # self.x_hat[4] = self.x_hat[4] - K_b * innovation / (self.dt ** 2)

        # update uncertainty
        self.P_x = (1-K_x) * self.P_x
        self.P_y = (1-K_y) * self.P_y
    

