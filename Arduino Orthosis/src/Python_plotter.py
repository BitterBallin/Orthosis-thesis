import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# === USER SETTINGS ===
port = 'COM12'
baudrate = 115200
max_lines = 1000
skip_first = 10
skip_last = 2
buffer_size = 1000

# === Setup ===
ser = serial.Serial(port, baudrate)
time.sleep(2)  # Let Arduino reset

error_vals = deque(maxlen=buffer_size)
control_vals = deque(maxlen=buffer_size)
position_vals = deque(maxlen=buffer_size)
target_vals = deque(maxlen=buffer_size)
time_vals = deque(maxlen=buffer_size)


plt.ion()
fig, ax = plt.subplots()

line_count = 0

while True:
    line = ser.readline().decode(errors='ignore').strip()

    if line_count < skip_first:
        line_count += 1
        continue

    if line_count >= (max_lines - skip_last):
        print("Reached max usable lines, exiting...")
        break

    try:
        parts = line.strip().split(",")

        if len(parts) != 5:
            raise ValueError("Line does not have exactly 5 parts")

        timestamp = float(parts[0])  # already in seconds
        error = float(parts[1]) * 1000
        control = float(parts[2]) / 100
        position = float(parts[3]) * 1000
        target = float(parts[4]) * 1000

        time_vals.append(timestamp)
        error_vals.append(error)
        control_vals.append(control)
        position_vals.append(position)
        target_vals.append(target)

        ax.clear()
        ax.plot(time_vals, error_vals, label='Error')
        ax.plot(time_vals, control_vals, label='Control')
        ax.plot(time_vals, position_vals, label='Position')
        ax.plot(time_vals, target_vals, label='Target Position')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distances in mm, control/100")
        
        plt.pause(0.001)  # <-- refresh plot window
        line_count += 1   # <-- only increment if valid data

        if line_count >= (max_lines - skip_last):
            print("Reached max usable lines, exiting...")
            break

    except ValueError as e:
        print(f"[Skipped] {line} → {e}")
        continue

plt.ioff()
plt.show()
ser.close()
