import numpy as np
import matplotlib.pyplot as plt

# Define coefficients of the quintic polynomial equation
q0 = 64
qf = 256

tf = 3


a0 = 0
a1 = 0
a2 = 0
a3 = 10*(qf-q0)/pow(tf, 3)
a4 = -15*(qf-q0)/pow(tf, 4)
a5 = 6*(qf-q0)/pow(tf, 5)

# Define time range
t = np.linspace(0, tf, 10000)  # Adjust time range as needed

# Compute position using the quintic polynomial equation
position = a5 * t**5 + a4 * t**4 + a3 * t**3 + a2 * t**2 + a1 * t + a0

# Compute velocity using the derivative of position
velocity = np.gradient(position, t)
velocity[0] = 0
velocity[len(position)-1] = 0

# Compute acceleration using the derivative of velocity
acceleration = np.gradient(velocity, t)
acceleration[0] = 0
acceleration[len(position)-1] = 0

# Compute jerk using the derivative of acceleration
jerk = np.gradient(acceleration, t)
for i in range(3):
    jerk[i] = None
    jerk[len(position)-1-i] = None

# Plot in subplots
fig, axs = plt.subplots(4,1, figsize=(8, 10))

# Plot position profile
axs[0].plot(t, position, color='blue')
axs[0].set_xlabel('time')
axs[0].set_ylabel('position')
axs[0].grid(True)

# Plot velocity profile
axs[1].plot(t, velocity, color='orange')
axs[1].set_xlabel('time')
axs[1].set_ylabel('velocity')
axs[1].grid(True)

# Plot acceleration profile
axs[2].plot(t, acceleration, color='green')
axs[2].set_xlabel('time')
axs[2].set_ylabel('acceleration')
axs[2].grid(True)

# Plot jerk profile
axs[3].plot(t, jerk, color='red')
axs[3].set_xlabel('time')
axs[3].set_ylabel('jerk')
axs[3].grid(True)

plt.tight_layout()
plt.show()
