import numpy as np
from tools.rotations import euler_to_quaternion, quaternion_to_euler, euler_to_rotation

# Problem 4
m = 25.0
fg_ned = m * np.array([0, 0, 9.81])

phi = np.radians(0.0)
theta = np.radians(7.0)
psi = np.radians(19.0)

R = euler_to_rotation(phi, theta, psi)

fg_body = R @ fg_ned

print(f"fg_body: {fg_body}")

a_meas = np.array([1.3, -3.2, -10.7])
F = fg_body + a_meas * m

print(f"F: {F}")