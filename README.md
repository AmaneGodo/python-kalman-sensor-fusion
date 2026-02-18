# Project #2 — 2D Bias-Aware Kalman Filter (IMU + GPS)
This project implements a bias-aware 2D state estimator that fuses IMU acceleration and GPS position to address two fundamental problems in robotics localization:
    - IMU drift caused by accelerometer bias and double integration
    - GPS noise and low update rate that make it unreliable for real-time motion tracking
The estimator demonstrates how sensor fusion and innovation-driven bias estimation produce a stable, bounded state estimate where either sensor alone fails.

## System Overview
The system is intentionally modular:

1. **Plant** — Simulated ground-truth motion with hidden disturbances
2. **IMU Sensor** — Measures acceleration with noise and constant bias
3. **GPS Sensor** — Measures absolute position with noise and lower frequency
4. **Estimator** — Bias-aware Kalman-style filter
5. **Simulation Loop** — Orchestrates timing, prediction, and correction

This separation mirrors real robotics software stacks and allows components to be modified or replaced independently.

## State Definition
The estimator tracks the following states per axis (x, y):

- **Position** — Quantity of interest
- **Velocity** — Required because position depends on integrated motion
- **Accelerometer Bias** — Hidden systematic error that causes IMU drift (Acceleration is treated as an input; accelerometer bias is modeled as a state)

## Prediction (IMU)
At each timestep, the estimator predicts motion using bias-corrected acceleration:
    accel_used = accel_measured - bias

- Velocity is updated from acceleration.
- Position is updated from velocity.
- Uncertainty grows due to noisy acceleration and integration.

Without correction, even a small bias causes velocity and position to diverge over time.

## Correction (GPS) & Innovation
When GPS data becomes available, the estimator computes the innovation:
    innovation = position_GPS - position_predicted

This value captures the disagreement between:
- IMU-based prediction (fast but biased)
- GPS measurement (unbiased but noisy)

### Effect of GPS Update Rate
To illustrate the impact of sensor timing, the simulation is executed under three GPS update regimes:
- Fast GPS      (Update interval = 0.5s) - frequent absolute correction
- Slow GPS      (Update interval = 2.5s) - moderate correction delay
- Very Slow GPS (Update interval = 5.0s) - Sparse absolute updates

Observations
- Fast GPS
    - Frequent corrections suppress IMU drift quickly
    - Bias converges smoothly
    - Estimated position closely tracks true motion

- Slow GPS
    - Larger prediction intervals allow bias-induced drift to accumulate
    - GPS corrections appear as discrete jumps in the estimate
    - Estimator remains stable but lags true motion

- Very Slow GPS
    - Estimator relies heavily on IMU prediction 
    - Larger correction jumps occur
    - Bias estimation becomes slower and noisier

This demonstrates that estimation quality depends not only on sensor accuracy, but also on sensor timing:
    Even an unbiased sensor becomes less informative if it updates too slowly. 

## Estimation-Control Interaction
A feedback controller uses the **Kalman-filtered state estimate** to compute control inputs.

As a reasult:
- The true state depends on estimation quality
- Estimation errors propagate into control actions
- Sensor fusion stability directly affects closed-loop behavior

This coupling reflects real robotic systems, where:
- Controllers never see ground truth
- Estimation errors directly affect motion

The system demonstrates that robust control is impossible without robust state estimation. 

### Key Insight
- Accelerometer bias is not directly measurable from GPS.
- It becomes observable only through persistent innovation over time.
    - Consistent positive innovation → prediction lagging → bias too negative
    - Consistent negative innovation → prediction overshooting → bias too positive
- The estimator uses this information to:
    - Correct position using a Kalman gain
    - Slowly adjust the bias estimate to suppress long-term drift

## Results
The simulation demonstrates IMU-only prediction, GPS-only correction, and fused estimation under closed-loop control:
- IMU-only: unbounded drift
- GPS-only: noisy, jittery position
- Fused estimate:
    - bounded position
    - suppressed drift
    - stable behavior despite noise and bias

Bias estimates converge gradually without instability, validating innovation-driven correction.

### Plots
All plots show true state, IMU dead-reckoning, GPS measurements, and the fused Kalman estimate

- Fast GPS (0.5s update)
![Fast GPS X](data/robotics/Sensor_Fusion/x_fast_gps.png)
![Fast GPS Y](data/robotics/Sensor_Fusion/y_fast_gps.png)
![Fast GPS Bias](data/robotics/Sensor_Fusion/bias_fast_gps.png)

**Observation:**  
Frequent GPS corrections rapidly suppress IMU drift. Bias converges smoothly and position closely tracks the true state.

- Slow GPS (2.5s update)
![Fast GPS X](data/robotics/Sensor_Fusion/x_slow_gps.png)
![Fast GPS Y](data/robotics/Sensor_Fusion/y_slow_gps.png)
![Fast GPS Bias](data/robotics/Sensor_Fusion/bias_slow_gps.png)

**Observation:**  
Longer prediction intervals allow drift to accumulate. Corrections appear as discrete jumps, but stability is maintained.

- Very Slow GPS (5.0s update)
![Fast GPS X](data/robotics/Sensor_Fusion/x_very_slow_gps.png)
![Fast GPS Y](data/robotics/Sensor_Fusion/y_very_slow_gps.png)
![Fast GPS Bias](data/robotics/Sensor_Fusion/bias_very_slow_gps.png)

**Observation:**  
Sparse GPS updates significantly reduce bias observability. Estimator relies heavily on IMU prediction, leading to larger correction steps.

## Design Decisions & Simplifications
- Scalar (decoupled) updates are used instead of a full covariance matrix.
- Focus is on observability, bias behavior, and estimator intuition.
- Cross-axis coupling and full matrix algebra are intentionally deferred.

## Limitations
These are conscious tradeoffs, not omissions.

- No full covariance matrix (F, P, Q, H)
- No delayed measurement handling
- Simple feedback controller (no optimal control)
- Simplified sensor models

## Planned Extensions
- Full matrix Kalman filter implementation in C++
- Explicit cross-covariabce handling
- Multithreaded sensor update loops
- Integration with control 
- Full matirx Kalman filter (F, P, Q, H)
- Out-of-sequence measurement handling

## Key Takeaway
This project demonstrates that:
    - Reliable localization requires both fast predicton and absolute correction, and that sensor bias can only be inferred indirectly through persistent innovation. 

## Technologies & Concepts
Kalman Filtering · Sensor Fusion · Inertial Navigation · Bias Estimation · Robotics Simulation · Python