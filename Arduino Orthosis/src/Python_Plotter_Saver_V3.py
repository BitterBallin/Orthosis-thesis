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
plot_every_n = 1  # Plot every Nth row from CSV

# PID Gains (set these to your actual values)
target_max = 0.08 #Peak target in meters
proportional = 4.5*255/target_max
integral = 3500
derivative = 40
tau_d = 0.015  # Filter time constant in seconds

# === Initialize serial ===
ser = serial.Serial(port, baudrate)
ser.reset_input_buffer()

# === Data buffers ===
time_vals = deque(maxlen=buffer_size)
angle_vals = deque(maxlen=buffer_size)
force_vals = deque(maxlen=buffer_size)
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
        if len(parts) != 5:
            raise ValueError(f"Line does not have exactly 5 parts, got {len(parts)}")

        timestamp = float(parts[0])
        angle = float(parts[1])  # degrees
        force = float(parts[2])
        position = float(parts[3])
        target = float(parts[4])

        time_vals.append(timestamp)
        angle_vals.append(angle)
        force_vals.append(force)
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
previous_position = position_vals[0]
previous_target = target_vals[0]
previous_time = time_vals[0]
previous_filtered_edot = 0
previous_error = -previous_position + previous_target

rpm_time = []
for i in range(1, len(time_vals)):
    t = time_vals[i]
    dt = t - previous_time
    if dt <= 0:
        continue

    position = position_vals[i]
    target = target_vals[i]
    error = -position + target

    # --- edot based on error ---
    dedt = -(error - previous_error) / dt
    alpha = dt / (tau_d + dt)
    filtered_edot = alpha * dedt + (1 - alpha) * previous_filtered_edot

    # if abs(error) < 0.00005:
    #     error_integral = 0
    #     # filtered_edot = 0

    decay_factor = 0.995  # closer to 1.0 = slow decay, < 0.99 = aggressive
    error_integral = error_integral * decay_factor + error * dt

    # error_integral += error * dt

    P = proportional * error
    I = integral * error_integral
    D = derivative * filtered_edot
    control = P + I + D

    error_vals.append(error)
    p_vals.append(P)
    i_vals.append(I)
    d_vals.append(D)
    control_vals.append(control)

    previous_time = t
    previous_position = position
    previous_filtered_edot = filtered_edot

    # --- RPM ---
    a1 = angle_vals[i - 1]
    a2 = angle_vals[i]
    delta_angle = (a2 - a1 + 180) % 360 - 180
    rpm = (delta_angle / dt) / 360 * 60
    rpm_vals.append(rpm)
    rpm_time.append(t)

# === Save to CSV ===
test_number = 2
current_date = datetime.datetime.now().date()
csv_filename = f"TB1_test_{test_number}_{current_date}.csv"

with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "Time (s)", "Angle (deg)", "Force (N)", "Position (m)", "Target (m)",
        "Error", "P", "I", "D", "Control", "RPM"
    ])
    for i in range(1, len(error_vals)):
        writer.writerow([
            round(time_vals[i], 5),
            round(angle_vals[i], 3),
            round(force_vals[i], 3),
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
position_vals = []
target_vals = []
control_vals = []
p_vals = []
i_vals = []
d_vals = []
rpm_vals = []

with open(csv_filename, mode='r') as file:
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

plt.axhline(25.5, color='gray', linestyle='--', linewidth=1)
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




# import serial
# import time
# import matplotlib.pyplot as plt
# from collections import deque
# import csv
# import datetime
# import numpy as np

# # === USER SETTINGS ===
# port = 'COM12'
# baudrate = 250000
# max_lines = 10000
# skip_first = 5
# skip_last = 2
# buffer_size = 10000
# plot_every_n = 5  # Plot every Nth row from CSV

# # === Initialize serial ===
# ser = serial.Serial(port, baudrate)
# ser.reset_input_buffer()

# # === Data buffers ===
# time_vals = deque(maxlen=buffer_size)
# force_vals = deque(maxlen=buffer_size)
# position_vals = deque(maxlen=buffer_size)
# target_vals = deque(maxlen=buffer_size)
# angle_vals = deque(maxlen=buffer_size)
# angle_time_vals = deque(maxlen=buffer_size)

# line_count = 0

# # === Start data acquisition ===
# while True:
#     line = ser.readline().decode(errors='ignore').strip()

#     if line_count < skip_first:
#         line_count += 1
#         continue

#     if "END" in line:
#         print("Arduino finished test.")
#         break

#     if line_count >= (max_lines - skip_last):
#         print("Reached max usable lines, exiting...")
#         break

#     try:
#         if line.startswith("CurrentAngle:"):
#             angle = float(line.replace("CurrentAngle:", "").strip())
#             timestamp = time.time()  # current system time in seconds
#             angle_vals.append(angle)
#             angle_time_vals.append(timestamp)
#             continue

#         parts = line.split(",")
#         if len(parts) != 5:
#             raise ValueError(f"Line does not have exactly 5 parts, got {len(parts)}")

#         timestamp = float(parts[0])
#         angle = float(parts[1])
#         force = float(parts[2])
#         position = float(parts[3])
#         target = float(parts[4])

#         time_vals.append(timestamp)
#         angle_vals.append(angle)
#         angle_time_vals.append(timestamp)
#         force_vals.append(force)
#         position_vals.append(position)
#         target_vals.append(target)


#         line_count += 1

#     except ValueError as e:
#         print(f"[Skipped] {line} → {e}")
#         continue

# ser.close()

# # === Save to CSV ===
# test_number = 2
# current_date = datetime.datetime.now().date()
# csv_filename = f"TB1_test_{test_number}_{current_date}.csv"
# with open(csv_filename, mode='w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(["Time (s)", "Force (N)", "Position (mm)", "Target (mm)"])
#     for i in range(len(time_vals)):
#         writer.writerow([
#             round(time_vals[i], 5),
#             round(force_vals[i], 5),
#             round(position_vals[i], 5),
#             round(target_vals[i], 5)
#         ])

# print(f"Data saved to '{csv_filename}'")

# # === Reload and downsample for plotting ===
# time_vals = []
# force_vals = []
# position_vals = []
# target_vals = []

# with open(csv_filename, mode='r') as file:
#     reader = csv.reader(file)
#     next(reader)  # Skip header

#     for row in reader:
#         try:
#             time_vals.append(float(row[0]))
#             force_vals.append(float(row[1]))
#             position_vals.append(float(row[2]))
#             target_vals.append(float(row[3]))
#         except ValueError:
#             continue

# # Downsample
# time_vals = time_vals[::plot_every_n]
# force_vals = force_vals[::plot_every_n]
# position_vals = position_vals[::plot_every_n]
# target_vals = target_vals[::plot_every_n]

# # === Calculate RPM (handling 0–360 wraparound) ===
# rpm_vals = []
# rpm_time = []

# for i in range(1, len(angle_vals)):
#     a1 = angle_vals[i - 1]
#     a2 = angle_vals[i]
#     t1 = angle_time_vals[i - 1]
#     t2 = angle_time_vals[i]

#     delta_time = t2 - t1
#     if delta_time <= 0:
#         continue

#     # Correct for wraparound: maps angle delta to [-180, 180]
#     delta_angle = (a2 - a1 + 180) % 360 - 180  # in degrees

#     # Convert degrees per second to RPM: (deg/s) / 360 * 60
#     rpm = (delta_angle / delta_time) / 360.0 * 60.0

#     rpm_vals.append(rpm)
#     rpm_time.append(t2 - angle_time_vals[0])  # relative time


# # === Plotting ===
# plt.close('all')

# # 1. Position vs Time
# plt.figure("Position vs Time")
# plt.plot(time_vals, position_vals, label='Position')
# plt.plot(time_vals, target_vals, label='Target', linestyle='--')
# plt.xlabel("Time (s)")
# plt.ylabel("Position (m)")
# plt.title("Position and Target Over Time")
# plt.legend()
# plt.grid(True)

# # 2. Force vs Time
# plt.figure("Force vs Time")
# plt.plot(time_vals, force_vals, label='Force', color='blue')
# plt.xlabel("Time (s)")
# plt.ylabel("Force (N)")
# plt.title("Force Over Time")
# plt.grid(True)

# # 3. Force vs Position
# plt.figure("Force vs Position")
# plt.plot(position_vals, force_vals, 'b-')
# plt.xlabel("Position (m)")
# plt.ylabel("Force (N)")
# plt.title("Force vs Position")
# plt.grid(True)

# # 4. RPM over Time
# if rpm_vals:
#     plt.figure("RPM over Time")
#     plt.plot(rpm_time, rpm_vals, label='RPM', color='green')
#     plt.xlabel("Time (s)")
#     plt.ylabel("RPM")
#     plt.title("Calculated RPM from Angle")
#     plt.grid(True)

# plt.show()
