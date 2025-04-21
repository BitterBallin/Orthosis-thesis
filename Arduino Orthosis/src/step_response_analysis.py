import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# Close all open figures
plt.close('all')

# === USER SETTINGS ===
port = 'COM12'
baudrate = 115200
max_lines = 2000
skip_first = 5
skip_last = 2
buffer_size = 2000

# === Setup ===
ser = serial.Serial(port, baudrate)
ser.reset_input_buffer()  # Flush old junk from buffer
import numpy as np
from scipy.optimize import curve_fit
import control as ctl

# --- Data acquisition ---
lines = []
print("Reading serial...")

while len(lines) < max_lines:
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if "END" in line:
                break
            if "," in line:
                lines.append(line)
        except UnicodeDecodeError:
            continue  # skip garbage lines

ser.close()
print(f"Read {len(lines)} lines.")

# --- Parse and clean data ---
data = []
for line in lines:
    try:
        t_str, angle_str = line.split(",")
        data.append([float(t_str), float(angle_str)])
    except ValueError:
        continue

data = np.array(data)
t = data[:, 0]
y = data[:, 1]

# Trim noisy start/stop if needed
t = t[skip_first: -skip_last]
y = y[skip_first: -skip_last]
t -= t[0]  # Normalize time to 0

# --- Plot raw step response ---
plt.figure()
plt.plot(t, y, label="Measured Step Response")
plt.xlabel("Time [s]")
plt.ylabel("Angle [rad]")
plt.title("DC Motor Step Response")
plt.grid(True)
plt.legend()

# --- Second-order system model ---
def second_order_step_response(t, K, zeta, omega_n):
    """Standard second-order step response (underdamped/overdamped supported)"""
    sys = ctl.TransferFunction([K * omega_n**2], [1, 2*zeta*omega_n, omega_n**2])
    t_out, y_out = ctl.step_response(sys, T=t)
    return y_out

# Initial guesses: [K (gain), zeta (damping), omega_n (natural freq)]
initial_guess = [y[-1], 0.5, 5]

# --- Fit model to data ---
try:
    popt, _ = curve_fit(second_order_step_response, t, y, p0=initial_guess)
    K_fit, zeta_fit, omega_n_fit = popt

    # Create the fitted transfer function
    num = [K_fit * omega_n_fit**2]
    den = [1, 2 * zeta_fit * omega_n_fit, omega_n_fit**2]
    tf_fit = ctl.TransferFunction(num, den)

    print("\n=== Fitted Transfer Function Parameters ===")
    print(f"Gain (K):      {K_fit:.4f}")
    print(f"Damping (ζ):   {zeta_fit:.4f}")
    print(f"ω_n (rad/s):   {omega_n_fit:.4f}")
    print(f"Transfer Function:\n{tf_fit}\n")

    # Plot fitted response
    t_sim, y_sim = ctl.step_response(tf_fit, T=t)
    plt.plot(t_sim, y_sim, '--', label="Fitted 2nd-Order Model")
    plt.legend()
    plt.show()

except RuntimeError as e:
    print("Curve fitting failed:", e)
