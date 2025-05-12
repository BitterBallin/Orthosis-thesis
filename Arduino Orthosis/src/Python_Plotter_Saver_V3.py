import serial
import time
import matplotlib.pyplot as plt
from collections import deque
import csv
import datetime
import numpy as np

# === USER SETTINGS ===
port = 'COM12'
baudrate = 250000
max_lines = 100000
skip_first = 5
skip_last = 2
buffer_size = max_lines
plot_every_n = 1

# PID Gains
target_max =5
proportional = 4.5 * 255 / target_max
integral = 20
derivative = 0.2
tau_d = 0.015

# === Initialize serial ===
ser = serial.Serial(port, baudrate)
ser.reset_input_buffer()

# === Data buffers ===
time_vals = deque(maxlen=buffer_size)
angle_vals = deque(maxlen=buffer_size)
force_vals = deque(maxlen=buffer_size)
tip_force_vals = deque(maxlen=buffer_size)
position_vals = deque(maxlen=buffer_size)
target_vals = deque(maxlen=buffer_size)

line_count = 0

# === Data acquisition ===
while True:
    line = ser.readline().decode(errors='ignore').strip()

    if line_count < skip_first:
        line_count += 1
        continue

    if "END" in line:
        print("Arduino finished test.")
        break

    if line_count >= (max_lines - skip_last):
        print("Reached max usable lines, exiting...")
        break

    try:
        parts = line.split(",")
        if len(parts) != 6:
            raise ValueError(f"Line does not have exactly 6 parts, got {len(parts)}")

        timestamp = float(parts[0])
        angle = float(parts[1])
        force = float(parts[2])
        tip_force = float(parts[3])
        position = float(parts[4])
        target = float(parts[5])

        time_vals.append(timestamp)
        angle_vals.append(angle)
        force_vals.append(force)
        tip_force_vals.append(tip_force)
        position_vals.append(position)
        target_vals.append(target)

        line_count += 1

    except ValueError as e:
        print(f"[Skipped] {line} → {e}")
        continue

ser.close()

# === Compute PID & RPM ===
error_vals = []
p_vals = []
i_vals = []
d_vals = []
control_vals = []
rpm_vals = []

error_integral = 0
previous_position = tip_force_vals[0]
previous_target = target_vals[0]
previous_time = time_vals[0]
previous_filtered_edot = 0
previous_error = -previous_position + previous_target
filtered_speed = 0.0
tau_speed = 0.02  # time constant in seconds
max_angular_speed = 20


rpm_time = []
for i in range(1, len(time_vals)):
    t = time_vals[i]
    dt = t - previous_time
    if dt <= 0:
        continue

    position = tip_force_vals[i]
    target = target_vals[i]
    error = -position + target

    dedt = -(error - previous_error) / dt
    alpha = dt / (tau_d + dt)
    filtered_edot = alpha * dedt + (1 - alpha) * previous_filtered_edot

    decay_factor = 0.995
    error_integral = error_integral * decay_factor + error * dt

    P = proportional * error
    I = integral * error_integral
    D = derivative * filtered_edot
    control = P + I + D

    # === Compute wrap-safe angular speed ===
    delta_deg = (angle_vals[i] - angle_vals[i - 1] + 540) % 360 - 180
    angle_delta_rad = np.deg2rad(delta_deg)
    raw_angular_speed = angle_delta_rad / dt  # rad/s

    # === Low-pass filter
    angle_alpha = dt / (tau_speed + dt)
    filtered_speed = angle_alpha * raw_angular_speed + (1 - angle_alpha) * filtered_speed

    # === Speed limiting with hysteresis
    hysteresis = 2.0  # rad/s tolerance
    if abs(filtered_speed) > max_angular_speed + hysteresis:
        scale = max_angular_speed / abs(filtered_speed)
        control *= scale * 0.2
        # print(f"[Speed-Limited] t={t:.3f}s | ω = {filtered_speed:.2f} rad/s | scale = {scale:.2f}")


    error_vals.append(error)
    p_vals.append(P)
    i_vals.append(I)
    d_vals.append(D)
    control_vals.append(control)

    previous_time = t
    previous_position = position
    previous_filtered_edot = filtered_edot

    a1 = angle_vals[i - 1]
    a2 = angle_vals[i]
    delta_angle = (a2 - a1 + 180) % 360 - 180
    rpm = (delta_angle / dt) / 360 * 60
    rpm_vals.append(rpm)
    rpm_time.append(t)

# === Save to CSV ===
test_number = 4
voltage = 30
wire_length = 155 # in cm
position_target = int(target_vals[len(target_vals) // 2])
current_date = datetime.datetime.now().date()
# csv_filename = f"TB1_test_{test_number}_{current_date}.csv"
# csv_filename = f"TB1_CHIRPTEST_TEST{test_number}_{voltage}V_{wire_length}CM_{current_date}.csv"
csv_filename = f"TB2_FINAL_FULLFLEXION_TARGET_{position_target}N_TEST_{test_number}_{voltage}V_{wire_length}CM_{current_date}.csv"
# csv_filename = f"TB2_SENSORDEBUGGING_RAMPTEST_ONESIDE_TARGET_{position_target}_TEST_{test_number}_{voltage}V_{wire_length}CM_{current_date}.csv"



with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "Time (s)", "Angle (deg)", "Force (N)", "Tip Force (N)",
        "Position (m)", "Target (m)", "Error", "P", "I", "D", "Control", "RPM"
    ])
    for i in range(1, len(error_vals)):
        writer.writerow([
            round(time_vals[i], 5),
            round(angle_vals[i], 3),
            round(force_vals[i], 3),
            round(tip_force_vals[i], 3),
            round(position_vals[i], 5),
            round(target_vals[i], 5),
            round(error_vals[i], 5),
            round(p_vals[i], 5),
            round(i_vals[i], 5),
            round(d_vals[i], 5),
            round(control_vals[i], 5),
            round(rpm_vals[i], 3)
        ])

print(f"Data saved to '{csv_filename}'")

# === Reload CSV for plotting ===
time_vals = []
force_vals = []
tip_force_vals = []
position_vals = []
target_vals = []
control_vals = []
p_vals = []
i_vals = []
d_vals = []
rpm_vals = []
angle_vals = []

with open(csv_filename, mode='r') as file:
    reader = csv.reader(file)
    next(reader)  # skip header
    for row in reader:
        try:
            time_vals.append(float(row[0]))          # Time
            angle_vals.append(float(row[1]))         # Angle
            force_vals.append(float(row[2]))         # Force
            tip_force_vals.append(float(row[3]))     # Tip Force
            position_vals.append(float(row[4]))      # Position
            target_vals.append(float(row[5]))        # Target
            error_vals.append(float(row[6]))         # Error
            p_vals.append(float(row[7]))             # P
            i_vals.append(float(row[8]))             # I
            d_vals.append(float(row[9]))             # D
            control_vals.append(float(row[10]))      # Control
            rpm_vals.append(float(row[11]))          # RPM
        except ValueError:
            continue


# Downsample
time_vals = time_vals[::plot_every_n]
force_vals = force_vals[::plot_every_n]
tip_force_vals = tip_force_vals[::plot_every_n]
position_vals = position_vals[::plot_every_n]
target_vals = target_vals[::plot_every_n]
p_vals = p_vals[::plot_every_n]
i_vals = i_vals[::plot_every_n]
d_vals = d_vals[::plot_every_n]
control_vals = control_vals[::plot_every_n]
rpm_vals = rpm_vals[::plot_every_n]

# === Plotting ===
plt.close('all')

# RMSE
squared_errors = [(p - t) ** 2 for p, t in zip(position_vals, target_vals)]
rmse = np.sqrt(np.mean(squared_errors))

plt.figure("Position vs Target with control signal")
plt.plot(time_vals, tip_force_vals, label='Fingertip Force (N))')
plt.plot(time_vals, target_vals, label='Force Target (N)', linestyle='--')
plt.plot(time_vals, [c / 100 for c in control_vals], label='Control Signal (scaled)', linestyle='-.')
plt.axhline(25.5, color='gray', linestyle='--', linewidth=1)
plt.axhline(-25.5, color='gray', linestyle='--', linewidth=1)
plt.xlabel("Time (s)")
plt.ylabel("Scaled Values")
plt.title(f"Control Signal, Position & Target (RMSE = {rmse * 1000:.2f} mm)")
plt.legend()
plt.grid(True)

plt.figure("PID Terms")
plt.plot(time_vals, p_vals, label='P')
plt.plot(time_vals, i_vals, label='I')
plt.plot(time_vals, d_vals, label='D')
plt.xlabel("Time (s)")
plt.ylabel("PID Contributions")
plt.title("P, I, D Terms Over Time")
plt.legend()
plt.grid(True)

plt.figure("Force vs Time")
plt.plot(time_vals, force_vals, 'b-')
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Force Over Time")
plt.grid(True)

plt.figure("Tip Force vs Time")
plt.plot(time_vals, tip_force_vals, 'r-')
plt.xlabel("Time (s)")
plt.ylabel("Tip Force (N)")
plt.title("Fingertip Force Over Time")
plt.grid(True)

plt.figure("RPM Over Time")
plt.plot(time_vals, rpm_vals, label='RPM', color='green')
plt.xlabel("Time (s)")
plt.ylabel("RPM")
plt.title("RPM vs Time")
plt.grid(True)

plt.show()
