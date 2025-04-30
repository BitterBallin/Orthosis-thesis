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

# Data buffers
error_vals = deque(maxlen=buffer_size)
control_vals = deque(maxlen=buffer_size)
position_vals = deque(maxlen=buffer_size)
target_vals = deque(maxlen=buffer_size)
time_vals = deque(maxlen=buffer_size)
p_vals = deque(maxlen=buffer_size)
i_vals = deque(maxlen=buffer_size)
d_vals = deque(maxlen=buffer_size)
force_vals = deque(maxlen=buffer_size) 


plt.ion()
fig, ax = plt.subplots()
fig_pid, ax_pid = plt.subplots()  # Separate plot for PID components
fig_force, ax_force = plt.subplots()

line_count = 0

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
            raise ValueError(f"Line does not have exactly got {len(parts)}")
        

        timestamp = float(parts[0])            # already in seconds
        error = float(parts[1]) * 1000
        force = float(parts[2])                # raw force value
        control = float(parts[3]) / 10
        position = float(parts[4]) * 1000
        target = float(parts[5]) * 1000
        p = float(parts[6])
        i = float(parts[7])
        d = float(parts[8])


        # Store values
        time_vals.append(timestamp)
        error_vals.append(error)
        force_vals.append(force)              # NEW
        control_vals.append(control)
        position_vals.append(position)
        target_vals.append(target)
        p_vals.append(p)
        i_vals.append(i)
        d_vals.append(d)

        # Main plot
        ax.clear()
        ax.plot(time_vals, error_vals, label='Error')
        ax.plot(time_vals, control_vals, label='Control')
        ax.plot(time_vals, position_vals, label='Position')
        ax.plot(time_vals, target_vals, label='Target')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distances in mm, control/100")
        ax.legend()

        # PID component plot
        ax_pid.clear()
        ax_pid.plot(time_vals, p_vals, label='P')
        ax_pid.plot(time_vals, i_vals, label='I')
        ax_pid.plot(time_vals, d_vals, label='D')
        ax.axhline(25.5, color='black', linestyle=':', linewidth=1, label='Max PWM')
        ax.axhline(-25.5, color='black', linestyle=':', linewidth=1)
        ax_pid.set_xlabel("Time (s)")
        ax_pid.set_ylabel("PID Contributions")
        ax_pid.legend()

        # === Force vs Position Plot ===
        ax_force.clear()
        ax_force.plot(position_vals, force_vals, 'b-')
        ax_force.set_xlabel("Position [mm]")
        ax_force.set_ylabel("Force [N]")  # Adjust unit label if needed
        ax_force.set_title("Force vs Position")
        ax_force.grid(True)

        plt.pause(0.001)
        line_count += 1

    except ValueError as e:
        print(f"[Skipped] {line} → {e}")
        continue

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

# === Final Outputs ===
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

plt.ioff()
plt.show(block=True)
ser.close()


import csv
import datetime
# === Save to CSV ===
test_number = 1

# Get the current date and time
current_datetime = datetime.datetime.now()
current_date = current_datetime.date()
csv_filename = f"TB1_test_{test_number}_{current_date}.csv"
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Error (mm)", "Control", "Position (mm)", "Target (mm)", "P", "I", "D", "Force (N)"])
    for i in range(len(time_vals)):
        writer.writerow([
            time_vals[i],
            error_vals[i],
            control_vals[i],
            position_vals[i],
            target_vals[i],
            p_vals[i],
            i_vals[i], 
            d_vals[i],
            force_vals[i]
        ])

print(f"Data saved to '{csv_filename}'")
