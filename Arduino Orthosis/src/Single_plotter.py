import serial
import time
import matplotlib.pyplot as plt
from collections import deque
import csv
import datetime
import numpy as np


# === Reload CSV for plotting ===
time_vals = []
force_vals = []
position_vals = []
target_vals = []
control_vals = []
p_vals = []
i_vals = []
d_vals = []
rpm_vals = []

plot_every_n = 1

with open(r"src/Test Results TB2/One Sided Test Final/TB2_FINAL_RAMPTEST_ONESIDE_TARGET_80mm_TEST_7_24V_155CM_2025-05-08.csv") as file:
    reader = csv.reader(file)
    next(reader)  # Skip header
    for row in reader:
        try:
            time_vals.append(float(row[0]))
            force_vals.append(float(row[2]))
            position_vals.append(float(row[3]))
            target_vals.append(float(row[4]))
            p_vals.append(float(row[6]))
            i_vals.append(float(row[7]))
            d_vals.append(float(row[8]))
            control_vals.append(float(row[9]))
            rpm_vals.append(float(row[10]))
        except ValueError:
            continue

# Downsample for plotting
time_vals = time_vals[::plot_every_n]
force_vals = force_vals[::plot_every_n]
position_vals = position_vals[::plot_every_n]
target_vals = target_vals[::plot_every_n]
p_vals = p_vals[::plot_every_n]
i_vals = i_vals[::plot_every_n]
d_vals = d_vals[::plot_every_n]
control_vals = control_vals[::plot_every_n]
rpm_vals = rpm_vals[::plot_every_n]

# === Plotting ===
plt.close('all')

# === RMSE calculation ===
squared_errors = [(p - t) ** 2 for p, t in zip(position_vals, target_vals)]
rmse = np.sqrt(np.mean(squared_errors))

# Control vs Position vs Target
plt.figure("Position vs Target with control signal")

plt.plot(time_vals, [p * 1000 for p in position_vals], label='Position (mm)')
plt.plot(time_vals, [t * 1000 for t in target_vals], label='Target (mm)', linestyle='--')
plt.plot(time_vals, [c / 10 for c in control_vals], label='Control Signal (scaled)', linestyle='-.')

plt.axhline(25.5, color='gray', linestyle='--', linewidth=1, label = 'Max PWM control signal [0-25]')
plt.axhline(-25.5, color='gray', linestyle='--', linewidth=1)

plt.xlabel("Time (s)")
plt.ylabel("Scaled Values")
plt.title(f"Control Signal, Position & Target (RMSE = {rmse * 1000:.2f} mm)")
plt.legend()
plt.grid(True)

# PID terms
plt.figure("PID Terms")
plt.plot(time_vals, p_vals, label='P')
plt.plot(time_vals, i_vals, label='I')
plt.plot(time_vals, d_vals, label='D')
plt.xlabel("Time (s)")
plt.ylabel("PID Contributions")
plt.title("P, I, D Terms Over Time")
plt.legend()
plt.grid(True)

# ✅ New: Force vs Time
plt.figure("Force vs Time")
plt.plot(time_vals, force_vals, 'b-')
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Force Over Time")
plt.grid(True)

# RPM over time
plt.figure("RPM Over Time")
plt.plot(time_vals, rpm_vals, label='RPM', color='green')
plt.xlabel("Time (s)")
plt.ylabel("RPM")
plt.title("RPM vs Time")
plt.grid(True)

plt.show()