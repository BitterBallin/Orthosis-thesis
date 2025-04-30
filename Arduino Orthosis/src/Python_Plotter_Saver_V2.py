import serial
import time
import matplotlib.pyplot as plt
from collections import deque
import csv
import datetime
import os

# === USER SETTINGS ===
port = 'COM12'
baudrate = 250000
max_lines = 10000
skip_first = 5
skip_last = 2
buffer_size = 10000
plot_every_n = 5  # Plot every Nth row from CSV

# === Initialize serial ===
ser = serial.Serial(port, baudrate)
ser.reset_input_buffer()

# === Data buffers ===
error_vals = deque(maxlen=buffer_size)
control_vals = deque(maxlen=buffer_size)
position_vals = deque(maxlen=buffer_size)
target_vals = deque(maxlen=buffer_size)
time_vals = deque(maxlen=buffer_size)
p_vals = deque(maxlen=buffer_size)
i_vals = deque(maxlen=buffer_size)
d_vals = deque(maxlen=buffer_size)
force_vals = deque(maxlen=buffer_size)

line_count = 0

# === Start data acquisition ===
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
        parts = line.strip().split(",")
        if len(parts) != 9:
            raise ValueError(f"Line does not have exactly 9 parts, got {len(parts)}")

        timestamp = float(parts[0])
        error = float(parts[1]) * 1000
        force = float(parts[2])
        control = float(parts[3]) / 10
        position = float(parts[4]) * 1000
        target = float(parts[5]) * 1000
        p = float(parts[6])
        i = float(parts[7])
        d = float(parts[8])

        time_vals.append(timestamp)
        error_vals.append(error)
        force_vals.append(force)
        control_vals.append(control)
        position_vals.append(position)
        target_vals.append(target)
        p_vals.append(p)
        i_vals.append(i)
        d_vals.append(d)

        line_count += 1

    except ValueError as e:
        print(f"[Skipped] {line} → {e}")
        continue

ser.close()

# === Settling Time Calculation ===
settling_threshold = 5  # mm
required_stable_points = 10

settling_time = None
final_error = error_vals[-1]

for i in range(len(error_vals) - required_stable_points):
    window = list(error_vals)[i:i + required_stable_points]
    if all(abs(e - final_error) <= settling_threshold for e in window):
        settling_time = time_vals[i]
        break

# === Final Stats ===
print("\n=== Final Values ===")
print(f"Time: {time_vals[-1]:.3f} s")
print(f"Error: {error_vals[-1]:.3f} mm")
print(f"Control: {control_vals[-1]:.3f}")
print(f"Position: {position_vals[-1]:.3f} mm")
print(f"Target: {target_vals[-1]:.3f} mm")
if settling_time is not None:
    print(f"Settling Time: {settling_time:.3f} seconds")
else:
    print("No settling time detected.")

# === Save to CSV ===
test_number = 2
current_date = datetime.datetime.now().date()
csv_filename = f"TB1_test_{test_number}_{current_date}.csv"
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Error (mm)", "Control", "Position (mm)", "Target (mm)", "P", "I", "D", "Force (N)"])
    for i in range(len(time_vals)):
        writer.writerow([
            round(time_vals[i], 5),
            round(error_vals[i], 5),
            round(control_vals[i], 5),
            round(position_vals[i], 5),
            round(target_vals[i], 5),
            round(p_vals[i], 5),
            round(i_vals[i], 5),
            round(d_vals[i], 5),
            round(force_vals[i], 5)
        ])

print(f"Data saved to '{csv_filename}'")

# === Now plot from the CSV file ===

# === Reload and downsample for plotting ===
time_vals = []
error_vals = []
control_vals = []
position_vals = []
target_vals = []
p_vals = []
i_vals = []
d_vals = []
force_vals = []

with open(csv_filename, mode='r') as file:
    reader = csv.reader(file)
    headers = next(reader)  # Skip header

    for row in reader:
        try:
            time_vals.append(float(row[0]))
            error_vals.append(float(row[1]))
            control_vals.append(float(row[2]))
            position_vals.append(float(row[3]))
            target_vals.append(float(row[4]))
            p_vals.append(float(row[5]))
            i_vals.append(float(row[6]))
            d_vals.append(float(row[7]))
            force_vals.append(float(row[8]))
        except ValueError:
            continue

# Downsample
time_vals = time_vals[::plot_every_n]
error_vals = error_vals[::plot_every_n]
control_vals = control_vals[::plot_every_n]
position_vals = position_vals[::plot_every_n]
target_vals = target_vals[::plot_every_n]
p_vals = p_vals[::plot_every_n]
i_vals = i_vals[::plot_every_n]
d_vals = d_vals[::plot_every_n]
force_vals = force_vals[::plot_every_n]

# === Plotting ===
plt.close('all')

# 1. Main Plot
plt.figure("Main Plot")
plt.plot(time_vals, error_vals, label='Error')
plt.plot(time_vals, control_vals, label='Control')
plt.plot(time_vals, position_vals, label='Position')
plt.plot(time_vals, target_vals, label='Target')
plt.xlabel("Time (s)")
plt.ylabel("Distances in mm, control/100")
plt.legend()
plt.title("Control & Position vs Time")
plt.grid(True)

# 2. PID Plot
plt.figure("PID Contributions")
plt.plot(time_vals, p_vals, label='P')
plt.plot(time_vals, i_vals, label='I')
plt.plot(time_vals, d_vals, label='D')
plt.axhline(25.5, color='black', linestyle=':', linewidth=1, label='Max PWM')
plt.axhline(-25.5, color='black', linestyle=':', linewidth=1)
plt.xlabel("Time (s)")
plt.ylabel("PID Contributions")
plt.legend()
plt.title("PID Terms Over Time")
plt.grid(True)

# 3. Force vs Position
plt.figure("Force vs Position")
plt.plot(position_vals, force_vals, 'b-')
plt.xlabel("Position [mm]")
plt.ylabel("Force [N]")
plt.title("Force vs Position")
plt.grid(True)

plt.show()
