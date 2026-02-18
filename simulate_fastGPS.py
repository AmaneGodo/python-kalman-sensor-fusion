import numpy as np
from matplotlib import pyplot as plt
from plant import Plant2D
from sensor import IMUSensor, GPSSensor
from estimator import KalmanFilter2D
from pathlib import Path

# Get directory where this script lives
BASE_DIR = Path(__file__).resolve().parent

# Create plots folder inside this project
PLOT_DIR = BASE_DIR / "plots"
PLOT_DIR.mkdir(exist_ok=True)

# -----------------------------
# Simulation parameters
# -----------------------------
dt = 0.1
sim_time = 20.0
steps = int(sim_time/dt)

imu_vx = 0
imu_vy = 0

imu_x = 0
imu_y = 0

# -----------------------------
# Create system components
# -----------------------------
plant = Plant2D(dt=dt, x0=0, y0=0, vx0=0, vy0=0, ax_true=0, ay_true=0, x_disturbance=-1.0, y_disturbance=-0.75)

imu = IMUSensor(bax_true=0.05, bay_true=0.08, noise_std=0.05)

gps = GPSSensor(noise_std=1, update_interval=5)

kalman = KalmanFilter2D(dt, x0=0, y0=0, vx0=0, vy0=0, bax=0, bay=0, Q_imu=0.01, Q_bias=0.001, R_gps=1)

# -----------------------------
#  Logging for plots
# -----------------------------
# true position - from the plant
x_true = []
y_true = []

# gps position - from GPS sensor, no bias but noisy
x_gps = []
y_gps = []

# imu dead-reckoning trajectory - from integrating IMU acceleration, smooth but drifts (-> bias)
x_imu = []
y_imu = []

# estimated values by Kalman filter
estimated_pos_x = []
estimated_pos_y = []
z_gps_x = None
z_gps_y = None

bias_est_x = []
bias_est_y = []

for _ in range(steps):
    # true positions
    x_true.append(plant.x)
    y_true.append(plant.y)

    ax_measure, ay_measure = imu.measure(0, 0)
    
    z_gps_x, z_gps_y = gps.measure(plant.x, plant.y)
    x_gps.append(z_gps_x)
    y_gps.append(z_gps_y)

    kalman.predict(ax_measure, ay_measure)

    if z_gps_x is not None and z_gps_y is not None:
        kalman.update_gps(z_gps_x, z_gps_y)

    estimated_pos_x.append(kalman.x_hat[0])
    estimated_pos_y.append(kalman.x_hat[1])

    imu_vx += ax_measure * dt
    imu_vy += ay_measure * dt

    imu_x += imu_vx * dt
    imu_y += imu_vy * dt

    x_imu.append(imu_x)
    y_imu.append(imu_y)
    
    # simplified controller
    ## desired position 
    x_desired = 0.0
    y_desired = 0.0

    ## PD gains (starting small)
    kp = 0.1
    kd = 0.05

    ## control law
    ### a_command = kp * (desired position - kalman estimated position) - kd * kalman estimated velocity
    ax_command = kp * (x_desired - kalman.x_hat[0]) - kd * kalman.x_hat[2]
    ay_command = kp * (y_desired - kalman.x_hat[1]) - kd * kalman.x_hat[3]

    # feed controller command into the plant
    plant.step(ax_command, ay_command)

    # bias estimate tracking
    bias_est_x.append(kalman.x_hat[4])
    bias_est_y.append(kalman.x_hat[5])


# -----------------------------
# Plot results
# -----------------------------
time = np.arange(steps) * dt

# -----------------------------
# Robotics behavior
# -----------------------------


# x-plot
plt.figure(figsize=(10, 5))
plt.plot(time, x_true, label="X True Position")
plt.plot(time, x_imu, label="X IMU Position")
plt.plot(time, estimated_pos_x, label="X estimated position from kalman")
plt.plot(time, x_gps, '.', alpha=0.4, label="X GPS (update interval = 0.5s) Position")
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.title("X plot of true vs IMU vs Fast GPS vs kalman estimated position")
plt.legend()
plt.grid()
out_path = PLOT_DIR / "x_fast_gps.png"
plt.savefig(out_path, dpi=300)
plt.show()

# y-plot
plt.figure(figsize=(10, 5))
plt.plot(time, y_true, label="Y True Position")
plt.plot(time, y_imu, label="Y IMU Position")
plt.plot(time, estimated_pos_y, label="Y estimated position from kalman")
plt.plot(time, y_gps, '.', alpha=0.4, label="Y GPS (update interval = 0.5s) Position")
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.title("Y plot of true vs IMU vs Fast GPS vs kalman estimated position")
plt.legend()
plt.grid()

out_path = PLOT_DIR / "y_fast_gps.png"
plt.savefig(out_path, dpi=300)
plt.show()

# both x and Y bias plot
plt.figure(figsize=(10, 5))
plt.plot(time, bias_est_x, label="Estimated Bias")
plt.plot(time, bias_est_y, label="Estimated Bias")
plt.xlabel("Time (s)")
plt.ylabel("m/s^2")
plt.title("Estimated bias over time with fast GPS")
plt.legend()
plt.grid()

out_path = PLOT_DIR / "bias_fast_gps.png"
plt.savefig(out_path, dpi=300)
plt.show()




