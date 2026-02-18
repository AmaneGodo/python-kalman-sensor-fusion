import numpy as np

class IMUSensor:
    """
    IMU-like accelerometer.

    Measures acceleration with:
    - additive noise
    - unknown constant or slowly drifting bias
    """

    def __init__(self, bax_true, bay_true, noise_std):
        self.bias_ax = bax_true
        self.bias_ay = bay_true
        self.noise_std = noise_std

    def measure(self, ax_true, ay_true):
        """
        Return noisy biased acceleration measurement.
        """
        # TODO: add bias + noise
        ax_measure = ax_true + self.bias_ax + np.random.normal(0, self.noise_std)
        ay_measure = ay_true + self.bias_ay + np.random.normal(0, self.noise_std)
        return ax_measure, ay_measure

class GPSSensor:
    """
    GPS-like position sensor.

    Low-rate, noisy position measurements.
    """

    def __init__(self, noise_std, update_interval):
        self.noise_std = noise_std
        self.update_interval = update_interval
        self.counter = 0

    def measure(self, x_true, y_true):
        """
        Return noisy position measurement.
        """
        self.counter += 1

        if self.counter % self.update_interval == 0:
            x_measure = x_true + np.random.normal(0, self.noise_std)
            y_measure = y_true + np.random.normal(0, self.noise_std)

            return x_measure, y_measure
        
        else:
            return None, None