# ufo_ekf.py
import numpy as np


class EkfStateObserver:
    # implement continuous-discrete EFK for UFO
    def __init__(self):
        
        self.m = 10.0
        self.b0 = 0.6
        self.b1 = 0.2
        self.g = 9.81
        self.Ts = 0.05
        # process and measurement noise
        self.Q = np.array([[1.0**2, 0.0],
                            [0.0, 0.1**2]])
        self.Qu = np.array([[1.0**2]])

        self.R = np.array([[0.3**2]])
        # initialize state and covariance
        self.vel0 = 0.0
        self.pos0 = 0.0
        self.xhat = np.array([ [self.vel0], [self.pos0] ])
        self.Px = np.eye(2)
        self.n = self.Px.shape[0]

    def update(self, inp):
        # ekf algorithm for ufo
        F = inp[0]
        z_m = inp[1]
        t = inp[2]
        if t == 90.65:
            print("t = 90.85")

        # prediction step
        N = 10
        for j in range(0, N):
            v, z = self.xhat.flatten()
            z_dot = v
            v_dot = (-self.b0 / self.m) * v - (self.b1 / self.m) * v**3 - self.g + (1 / self.m) * F
            self.xhat = self.xhat + self.Ts/N * np.array([[v_dot], [z_dot]])


            A = np.array([
                [-self.b0 / self.m + (3 * self.b1 / self.m) * v**2, 0.0],
                [1.0, 0.0]
            ])
            Ad = np.eye(2) + A * (self.Ts/N) + A @ A * (self.Ts/N)**2 / 2
            self.Px = Ad @ self.Px @ Ad.T + (self.Ts/N)**2 * self.Q          

        C = np.array([[0, 1]])
        S_inv = np.linalg.inv(self.R + C @ self.Px @ C.T)
        L = self.Px @ C.T @ S_inv
        self.Px = (np.eye(2) - L @ C) @ self.Px @ (np.eye(2) - L @ C).T + L @ self.R @ L.T
        test = np.array([z_m - self.xhat[1]])
        self.xhat = self.xhat + L @ test
        return self.xhat

    

