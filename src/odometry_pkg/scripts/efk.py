# ekf.py
import numpy as np
from utils import IMUListener

imu_listener = IMUListener()

def compute_F(x, u, imu_data, dt):
    v = u[0]
    theta = x[2]
    a_x = imu_listener['accel_filtered']['x']
    a_y = imu_listener['accel_filtered']['y']

    F = np.array([
        [1, 0, -v * np.sin(theta) * dt, dt, 0, 0],
        [0, 1,  v * np.cos(theta) * dt, 0, dt, 0],
        [0, 0, 1,                       0, 0, dt],
        [0, 0, -a_x * dt * np.sin(theta), 1, 0, 0],
        [0, 0,  a_y * dt * np.cos(theta), 0, 1, 0],
        [0, 0, 0,                       0, 0, 1]
    ])
    return F

def predict_state(x, u, imu_data, L, dt):

    v = u[0]
    # delta se incluye en u pero no se usa explícitamente en este modelo
    theta = x[2]
    a_x = imu_listener['accel_filtered']['x']
    a_y = imu_listener['accel_filtered']['y']
    omega = imu_listener['gyro_filtered']['z']

    x_next = np.array([
        x[0] + x[3] * dt,
        x[1] + x[4] * dt,
        theta + omega * dt,
        v + a_x * dt * np.cos(theta),
        v + a_y * dt * np.sin(theta),
        omega
    ])
    return x_next