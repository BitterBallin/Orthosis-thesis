import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# Close all open figures
plt.close('all')

# === USER SETTINGS ===
port = 'COM12'
baudrate = 115200
max_lines = 10000
skip_first = 5
skip_last = 2
buffer_size = 10000

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

plot_every_n = 20  # Plot every Nth data point (adjust this value)


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


        # Plotting every Nth point only
        if line_count % plot_every_n == 0:
            # === Downsample for plotting ===
            skip = plot_every_n
            times_ds = list(time_vals)[::skip]
            error_ds = list(error_vals)[::skip]
            control_ds = list(control_vals)[::skip]
            position_ds = list(position_vals)[::skip]
            target_ds = list(target_vals)[::skip]
            p_ds = list(p_vals)[::skip]
            i_ds = list(i_vals)[::skip]
            d_ds = list(d_vals)[::skip]
            force_ds = list(force_vals)[::skip]

            # === Main Plot ===
            ax.clear()
            ax.plot(times_ds, error_ds, label='Error')
            ax.plot(times_ds, control_ds, label='Control')
            ax.plot(times_ds, position_ds, label='Position')
            ax.plot(times_ds, target_ds, label='Target')
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Distances in mm, control/100")
            ax.legend()

            # === PID Plot ===
            ax_pid.clear()
            ax_pid.plot(times_ds, p_ds, label='P')
            ax_pid.plot(times_ds, i_ds, label='I')
            ax_pid.plot(times_ds, d_ds, label='D')
            ax_pid.axhline(25.5, color='black', linestyle=':', linewidth=1, label='Max PWM')
            ax_pid.axhline(-25.5, color='black', linestyle=':', linewidth=1)
            ax_pid.set_xlabel("Time (s)")
            ax_pid.set_ylabel("PID Contributions")
            ax_pid.legend()

            # === Force vs Position Plot ===
            ax_force.clear()
            ax_force.plot(position_ds, force_ds, 'b-')
            ax_force.set_xlabel("Position [mm]")
            ax_force.set_ylabel("Force [N]")
            ax_force.set_title("Force vs Position")
            ax_force.grid(True)

            plt.pause(0.001)

        # Now always increment line count
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
test_number = 2

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
