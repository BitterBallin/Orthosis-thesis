import numpy as np
import matplotlib.pyplot as plt
import math



#Defining target position early so it can be used for PID to PWM scaling
targetPosition = 0.08 #80 mm displacement target for full ROM


#Transfer function of wire
#========================================================
# --- Static wire characteristics ---
K = 300000          # [N/m] stiffness of wire
Fi = 200.0              # [N] static load
braid_factor = 1.0     # compensate for braid length
L = 1.55 * braid_factor  # original wire length [m]
Lc = L + Fi / K        # wire length under load [m]
r0 = 0.28e-3           # wire radius [m]

X_current = Lc     # initialize X to static length
rvar = r0          # initialize variable radius

n = 1000 #simulation steps

max_rotations = 500 #max rotations of wire
theta_vec = np.linspace(0, max_rotations*2*math.pi, n)
DX_vec = []

for i, theta in enumerate(theta_vec):
    #Compute amount of rotatios necessary for certain position
    if i == 0:
        X = Lc
        rvar = r0
        DX = 0
        DX_vec.append(DX)
    else:
        rvar = r0*np.sqrt(Lc/X)
        DX = Lc - np.sqrt(Lc**2 - (theta**2) * (rvar**2))
        DX_vec.append(DX)
    if DX > targetPosition:
        break


# Ensure theta_vec is the same length as DX_vec for further calculation
theta_vec_trim = theta_vec[:len(DX_vec)]
DX_vec = np.array(DX_vec)

# Calculate the first derivative d(DX)/d(theta)
dDX_dtheta = np.gradient(DX_vec, theta_vec_trim)

# Plotting the original DX and its reverse
plt.figure(figsize=(10, 6))
plt.plot(theta_vec_trim, DX_vec, label='DX vs Theta wire 1')
plt.plot(theta_vec_trim, DX_vec[::-1], 'r', label='DX vs Theta wire 2')
plt.xlabel('Theta [rad]')
plt.ylabel('DX [m]')
plt.title('Change in Wire Length (DX) vs. Wire Rotation (Theta)')
plt.grid()
plt.legend()

# Plotting the first derivative
plt.figure(figsize=(10, 6))
plt.plot(theta_vec_trim, dDX_dtheta*1000, label='d(DX)/d(Theta) wire 1')
plt.plot(theta_vec_trim, dDX_dtheta[::-1]*1000, 'r', label='d(DX)/d(Theta) wire 2 ')
plt.xlabel('Theta [rad]')
plt.ylabel('d(DX)/d(Theta) [mm/rad]')
plt.title('First Derivative of Change in Wire Length vs. Rotation')
plt.grid()
plt.legend()
plt.show()