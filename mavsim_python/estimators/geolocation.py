"""
target geolocation algorithm
    - Beard & McLain, PUP, 2012
    - Updated:
        4/1/2022 - RWB
        4/6/2022 - RWB
        7/13/2023 - RWB
        4/7/2025 - TWM
"""
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.camera_parameters as CAM
from tools.rotations import euler_to_rotation
from estimators.filters import ExtendedKalmanFilterContinuousDiscrete

# Note that state equations assume a constant-velocity model for the target
class Geolocation:
    def __init__(self, ts: float=0.01):
        self.ekf = ExtendedKalmanFilterContinuousDiscrete(
            f=self.f, 
            Q = 0.01 * np.diag([
                (1)**2,   # target north position
                (1)**2,   # target east position
                (1)**2,   # target down position
                (10)**2,  # target north velocity
                (10)**2,  # target east velocity
                (10)**2,  # target down velocity
                (3)**2,   # distance to target L
                ]),
            P0= 0.1*np.diag([
                10**2,  # target north position
                10**2,  # target east position
                10**2,  # target down position
                10**2,  # target north velocity
                10**2,  # target east velocity
                10**2,  # target down velocity
                10**2,  # distance to target L
                ]), 
            xhat0=np.array([[
                0.,  # target north position
                0.,  # target east position
                0.,  # target down position
                0.,  # target north velocity
                0.,  # target east velocity
                0.,  # target down velocity
                100.,  # distance to target L
                ]]).T, 
            Qu=0.01*np.diag([
                1**2, # mav north position
                1**2, # mav east position
                1**2, # mav down position
                1**2, # mav north velocity
                1**2, # mav east velocity
                1**2, # mav down velocity
                ]), 
            Ts = ts,
            N = 10
        )
        self.R = 0.1 * np.diag([1.0, 1.0, 1.0, 1.0])

    def update(self, mav, pixels):
        # system input is mav state
        u = np.array([
            mav.north,
            mav.east,
            -mav.altitude,
            mav.Va*np.cos(mav.chi),  # north velocity
            mav.Va*np.sin(mav.chi),  # east velocity
            0.0  # assume flat earth → zero vertical velocity
            ])
   
        xhat, P = self.ekf.propagate_model(u)
        # update with pixel measurement
        y=self.process_measurements(mav, pixels)
        xhat, P = self.ekf.measurement_update(
            y=y, 
            u=u,
            h=self.h,
            R=self.R)
        return xhat[0:3, :]  # return estimated NED position

    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        # system dynamics for propagation model: xdot = f(x, u)
        # x = [p_n, p_e, p_d, v_n, v_e, v_d, L]  (target position, velocity, and slant range)
        # u = [mav_n, mav_e, mav_d, mav_vn, mav_ve, mav_vd]  (used only for measurement model here)

        # 1. Target position rate of change is current velocity
        target_position_dot = x[3:6]  # dx/dt = v

        # 2. Target velocity is assumed constant (i.e., zero acceleration)
        target_velocity_dot = np.zeros((3, 1))

        # 3. Distance to target (slant range) is assumed constant
        L_dot = np.zeros((1, 1))

        # Combine all into xdot
        xdot = np.concatenate((target_position_dot, target_velocity_dot, L_dot), axis=0)
        return xdot


    def h(self, x:np.ndarray, u:np.ndarray)->np.ndarray:
        # measurement model y
            ######## TODO ########
        target_position = x[0:3]  # target position
        L = x[6].reshape((1, 1))  # slant range
        y = np.concatenate((target_position, L), axis=0)
        return y

    def process_measurements(self, mav, pixels):
        # Camera model to infer line-of-sight vector from pixel coordinates
        h = mav.altitude
        mav_position = np.array([[mav.north], [mav.east], [-h]])

        # Line-of-sight vector in camera frame
        ell = np.array([[pixels.pixel_x], [pixels.pixel_y], [CAM.f]])
        ell_c = ell / np.linalg.norm(ell)

        # Rotation from inertial to body frame
        R_b_i = euler_to_rotation(mav.roll, mav.pitch, mav.yaw).T

        # Rotation from gimbal to body frame (fixed)
        R_g_b = np.array([
            [np.cos(mav.gimbal_el) * np.cos(mav.gimbal_az), -np.sin(mav.gimbal_az), np.sin(mav.gimbal_el) * np.cos(mav.gimbal_az)],
            [np.cos(mav.gimbal_el) * np.sin(mav.gimbal_az),  np.cos(mav.gimbal_az), np.sin(mav.gimbal_el) * np.sin(mav.gimbal_az)],
            [-np.sin(mav.gimbal_el),                         0,                     np.cos(mav.gimbal_el)]
        ])

        # Rotation from camera to gimbal frame (fixed)
        R_c_g = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 0]
        ])

        # Total transformation to inertial frame
        ell_i = R_b_i @ R_g_b @ R_c_g @ ell_c

        # Slant range (projected down)
        L = h / -ell_i[2, 0]

        # Estimate of target location
        target_position = mav_position + L * ell_i

        # Return full measurement vector [n; e; d; L]
        y = np.concatenate((target_position, np.array([[L]])), axis=0)
        return y
