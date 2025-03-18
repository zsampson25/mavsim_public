import numpy as np
import matplotlib.pyplot as plt

class AutoPilot:
    """Altitude Hold Autopilot with Successive Loop Closure"""
    def __init__(self, Ts):
        # Controller Gains
        self.kd_th = -1.409  # Pitch derivative gain (k_dθ)
        self.kp_th = -3.716  # Pitch proportional gain (k_pθ)
        self.kp_h = 0.000388095  # Altitude proportional gain (k_ph)
        self.ki_h = 0.000215608  # Altitude integral gain (k_ih)
        
        # Aircraft Model Constants
        self.a_theta1 = 0.668  # Pitch system damping coefficient
        self.a_theta2 = 1.27  # Pitch system natural frequency squared
        self.a_theta3 = -2.08  # Pitch control effectiveness
        self.Va = 830.0  # Airspeed (ft/s)
        
        # Time Step
        self.Ts = Ts  

        # Integral and Derivative Storage
        self.integrator_h = 0.0  # Altitude integrator
        self.error_d1_h = 0.0  # Previous altitude error
        self.h_d1 = 0.0  # Previous altitude
        self.theta_d1 = 0.0  # Previous theta
        self.q = 0.0  # Initial pitch rate

        # Dirty Derivative Settings for Pitch Rate Estimation
        self.sigma = 0.1  # Filter coefficient
        self.q_dot = 0.0  # Estimated pitch rate

    def update(self, h_c, h, theta):
        """
        Computes the elevator deflection based on altitude and pitch control loops.
        Inputs:
            h_c - commanded altitude (ft)
            h  - current altitude (ft)
            theta - current pitch angle (rad)
        Output:
            delta_e - elevator deflection command (rad)
        """

        # --- Altitude Control (PI) ---
        error_h = h_c - h  # Altitude error
        self.integrator_h += self.Ts / 2.0 * (error_h + self.error_d1_h)  # Trapezoidal integration
        theta_c = self.kp_h * error_h + self.ki_h * self.integrator_h  # Commanded pitch angle
        
        # Store for next iteration
        self.error_d1_h = error_h
        self.h_d1 = h

        # --- Dirty Derivative for Pitch Rate (q) Estimation ---
        self.q_dot = (self.sigma * (theta - self.theta_d1) / (self.sigma + self.Ts))  # Estimate pitch rate
        self.theta_d1 = theta  # Store previous theta for next step

        # --- Pitch Control (PD) ---
        error_theta = theta_c - theta  # Pitch error
        delta_e = self.kp_th * error_theta + self.kd_th * self.q  # Elevator command

        # --- Pitch Dynamics (Simulating q response) ---
        q_dot = (self.a_theta3 * delta_e - self.a_theta1 * self.q - self.a_theta2 * theta) * self.Ts
        self.q += q_dot  # Integrate q
        
        return delta_e, theta_c, self.q
