import numpy as np
from tools.rotations import euler_to_quaternion, quaternion_to_euler, euler_to_rotation

# Problem 2

Vb = np.array([25.0, 1.0, -3.0])
phi = np.radians(-10.0)
theta = np.radians(5.0)
psi = np.radians(105.0)

R = euler_to_rotation(phi, theta, psi)

Vg = R @ Vb
Vw = np.array([2.0, -5.0, -1.0])
Va = Vg - Vw
print(f"Va: {Va}")

Va_magnitude = np.linalg.norm(Va)
print(f"Va_magnitude: {Va_magnitude}")
Va_body = R.T @ Va

angle_of_attack = np.degrees(np.arctan(Va_body[2] / Va_body[0]))
print(f"angle_of_attack: {angle_of_attack}")

sideslip_angle = np.degrees(np.arcsin(Va_body[1] / np.sqrt(Va_body[0]**2 + Va_body[2]**2)))
print(f"sideslip_angle: {sideslip_angle}")

flight_path_angle = theta - np.radians(angle_of_attack)
flight_path_angle = np.degrees(flight_path_angle)
print(f"flight_path_angle: {flight_path_angle}")