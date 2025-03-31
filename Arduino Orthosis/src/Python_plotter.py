import serial
import matplotlib.pyplot as plt
from collections import deque

# === USER SETTINGS ===
port = 'COM12'             # Replace with your actual port
baudrate = 115200
max_lines = 1000          # Total lines to read
skip_first = 20           # Lines to skip at the beginning
skip_last = 20            # Lines to ignore at the end (we just stop early)
buffer_size = 1000         # How much to show in plot window

# === Setup ===
ser = serial.Serial(port, baudrate)
error_vals = deque(maxlen=buffer_size)
control_vals = deque(maxlen=buffer_size)
position_vals = deque(maxlen=buffer_size)

plt.ion()
fig, ax = plt.subplots()

line_count = 0

while True:
    line = ser.readline().decode(errors='ignore').strip()
    print("Raw line:", line)
    # print("Parsed:", error_vals, control_vals, position_vals)

    if line_count < skip_first:
        line_count += 1
        continue

    if line_count >= (max_lines - skip_last):
        print("Reached max usable lines, exiting...")
        break

    try:
        error, control, position = map(float, line.split(","))
        error_vals.append(error*1000)
        control_vals.append(control/100)
        position_vals.append(position*1000)

        ax.clear()
        ax.plot(error_vals, label='Error')
        ax.plot(control_vals, label='Control Signal')
        ax.plot(position_vals, label='Position')
        ax.legend()
        plt.pause(0.001)

        line_count += 1

    except ValueError:
        # Ignore malformed lines
        continue

plt.ioff()
plt.show()

ser.close()
