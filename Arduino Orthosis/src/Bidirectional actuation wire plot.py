import numpy as np
import matplotlib.pyplot as plt
import math

# Defining target position early so it can be used for PID to PWM scaling
targetPosition = 0.08  # 80 mm displacement target for full ROM

# Transfer function of wire
#========================================================
# --- Static wire characteristics ---
K = 300000          # [N/m] stiffness of wire
Fi = 200.0          # [N] static load
braid_factor = 1.0
L = 1.55 * braid_factor  # original wire length [m]
Lc = L + Fi / K          # wire length under load [m]
r0 = 0.28e-3             # wire radius [m]

X_current = Lc
rvar = r0

n = 1000  # simulation steps
max_rotations = 500  # max rotations of wire
theta_vec = np.linspace(0, max_rotations * 2 * math.pi, n)
DX_vec = []

for i, theta in enumerate(theta_vec):
    if i == 0:
        X = Lc
        rvar = r0
        DX = 0
        DX_vec.append(DX)
    else:
        rvar = r0 * np.sqrt(Lc / X)
        DX = Lc - np.sqrt(Lc**2 - (theta**2) * (rvar**2))
        DX_vec.append(DX)
    if DX > targetPosition:
        break

# Trim vectors to same length
theta_vec_trim = theta_vec[:len(DX_vec)]
DX_vec = np.array(DX_vec)

# =========================
# Plot 1: Non-shifted wire 1 + wire 2
# =========================
plt.figure(figsize=(5, 4))
plt.plot(theta_vec_trim, DX_vec, label='Contraction of wire 1')
plt.plot(theta_vec_trim, DX_vec[::-1], 'r', label='Contraction of wire 2')
plt.plot(theta_vec_trim, DX_vec + DX_vec[::-1], 'k--', label='Contraction of wire 1 + 2')
plt.xlabel('Theta [rad]')
plt.ylabel('Contraction [m]')
plt.title('Contraction vs. Wire Rotation (Theta)')
plt.grid()
plt.legend()
plt.tight_layout()

# =========================
# Plot 2: Shifted wire 1 + 2 (overlap only)
# =========================
from scipy.interpolate import interp1d

# Shift wire 2 (DX_vec[::-1]) by +500 rad
theta_shifted = theta_vec_trim + 500

# Interpolate wire 1 (static) and wire 2 (shifted)
interp_wire1 = interp1d(theta_vec_trim, DX_vec, bounds_error=False, fill_value=np.nan)
interp_wire2 = interp1d(theta_shifted, DX_vec[::-1], bounds_error=False, fill_value=np.nan)

# Define overlapping theta range (where both curves exist)
theta_min = max(theta_vec_trim[0], theta_shifted[0])
theta_max = min(theta_vec_trim[-1], theta_shifted[-1])
theta_overlap = theta_vec_trim[(theta_vec_trim >= theta_min) & (theta_vec_trim <= theta_max)]

# Evaluate interpolated values at overlapping thetas
DX1_overlap = interp_wire1(theta_overlap)
DX2_overlap = interp_wire2(theta_overlap)

# Mask valid values (just in case)
valid_mask = ~np.isnan(DX1_overlap) & ~np.isnan(DX2_overlap)
theta_overlap_valid = theta_overlap[valid_mask]
DXsum_valid = DX1_overlap[valid_mask] + DX2_overlap[valid_mask]

# Plot
plt.figure(figsize=(5, 4))
plt.plot(theta_vec_trim, DX_vec, label='Contraction of wire 1')
plt.plot(theta_shifted, DX_vec[::-1], 'r', label='Contraction of wire 2')
plt.plot(theta_overlap_valid, DXsum_valid, 'k--', label='Contraction of wire 1 + 2 ')
plt.xlabel('Theta [rad]')
plt.ylabel('Contraction [m]')
plt.title('Contraction vs. Wire Rotation (Theta) with prewound Wire 2')
plt.grid()
plt.legend(loc = 'center left')
plt.tight_layout()



# =========================
# Plot 3: First derivative
# =========================
dDX_dtheta = np.gradient(DX_vec, theta_vec_trim)

plt.figure(figsize=(5, 4))
plt.plot(theta_vec_trim, dDX_dtheta * 1000, label='d(Contraction)/d(Theta) wire 1')
plt.plot(theta_vec_trim, dDX_dtheta[::-1] * 1000, 'r', label='d(Contraction)/d(Theta) wire 2')
plt.xlabel('Theta [rad]')
plt.ylabel('d(Contraction)/d(Theta) [mm/rad]')
plt.title('First Derivative of Contraction to Rotation')
plt.grid()
plt.legend()
plt.tight_layout()

plt.show()
