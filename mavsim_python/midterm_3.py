import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# Define numerator and denominator coefficients
a1 = 2.4897
a0 = 1.3831905
b3 = 3.598
b2 = 8.999928
b1 = 0
b0 = 0

num = [a1, a0]  # First-order numerator
den = [1, b3, b2, b1, b0]  # Fourth-order denominator

# Create transfer function
G = ctrl.TransferFunction(num, den)

# Time vector
t = np.linspace(0, 100000, 1000)  # Adjust duration as needed

# Step response for 100 ft step input
t_step, y_step = ctrl.step_response(100 * G, T=t)

# Plot response
plt.figure()
plt.plot(t_step, y_step, label="Altitude Response")
plt.title('Altitude Step Response')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (ft)')
plt.grid()
plt.legend()
plt.show()
