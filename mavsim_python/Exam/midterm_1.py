import numpy as np
from tools.rotations import euler_to_quaternion, quaternion_to_euler, euler_to_rotation

# Problem 1

phi = -25.0 * np.pi / 180.0
theta = -10.0 * np.pi / 180.0
psi = 110 * np.pi / 180.0

quaternion = euler_to_quaternion(phi, theta, psi)
print(f"quaternion: {quaternion}")

R = euler_to_rotation(phi, theta, psi)
V_body = np.array([35, -2, -5])

V_ned = R @ V_body
Vn = V_ned[0]
Ve = V_ned[1]
Vd = V_ned[2]

print(f"North: {Vn}, East: {Ve}, Down: {Vd}")