import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, step

wn_th = 3.0
wn_h = wn_th / 20.0

zeta_th = 0.6
zeta_h = 0.9

Va = 830.0

# Constants
a3_th = -2.08
a2_th = 1.27
a1_th = 0.668

kp_th = (wn_th**2 - a2_th) / a3_th
kd_th = (2.0 * zeta_th * wn_th - a1_th) / a3_th
K_dc_th = kp_th*a3_th / wn_th**2

print(f"kp_th: {kp_th}")
print(f"kd_th: {kd_th}")
print(f"K_dc_th: {K_dc_th}")

ki_h = wn_h**2 / (K_dc_th * Va)
kp_h = (2*zeta_h*wn_h) / (K_dc_th * Va)

print(f"ki_h: {ki_h}")
print(f"kp_h: {kp_h}")

# theta / theta_c

a0_th = kp_th*a3_th
b1_th = a1_th + kd_th*a3_th
b0_th = a2_th + kp_th*a3_th

print(f"a0_th: {a0_th}")
print(f"b1_th: {b1_th}")
print(f"b0_th: {b0_th}")

a1 = 1.583
a0 = .132
b3 = 1
b2 = 3.62
b1 = 9
b0 = 1.5833
b0_2 = .132

print(f"a1: {a1}")
print(f"a0: {a0}")

print(f"b3: {b3}")
print(f"b2: {b2}")
print(f"b1: {b1}")
print(f"b0: {b0}")
print(f"b0_2: {b0_2}")

num = [a1, a0]
den = [b3, b2, b1, b0, b0_2]


sys = TransferFunction(num, den)
t = np.linspace(0, 100, 1000)
t, y = step(sys, T=t)
y_scaled = 100 * y


plt.figure(figsize=(8, 5))
plt.plot(t, y_scaled, label="Step Response (Input = 100)")
plt.xlabel("Time (s)")
plt.ylabel("Response")
plt.title("Step Response of the System")
plt.grid()
plt.legend()
plt.show()
