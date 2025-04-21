import serial
import time
import matplotlib.pyplot as plt
import csv

# === USER SETTINGS ===
port = 'COM12'
baudrate = 115200
max_lines = 2000  # max lines to read
testnmbr = 1
output_csv = f'TestBench1Test{testnmbr}.csv'  # filename for saving
timeout_s = 20  # max seconds to wait for data (or until "END")

# === SETUP SERIAL ===
ser = serial.Serial(port, baudrate, timeout=1)
ser.reset_input_buffer()
print(f"Listening to {port}...")

# === DATA COLLECTION ===
data = []
start_time = time.time()

while True:
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if "END" in line:
                print("End of transmission received.")
                break

            # Parse comma-separated values
            parts = line.split(",")
            if len(parts) == 4:
                time_val = float(parts[0])
                force_val = float(parts[1])
                position_val = float(parts[2])
                angle_val = float(parts[3])
                data.append([time_val, force_val, position_val, angle_val])

        except Exception as e:
            print(f"Skipping line due to error: {e}")
            continue

    # Timeout in case "END" isn't received
    if time.time() - start_time > timeout_s:
        print("Timeout reached, stopping.")
        break

ser.close()

# === SAVE TO CSV ===
print(f"Saving {len(data)} rows to {output_csv}...")
with open(output_csv, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Force (N)", "Position (m)", "Angle (rad)"])  # header
    writer.writerows(data)

print("Done. CSV saved.")

# === OPTIONAL: QUICK PLOT ===
import numpy as np
data = np.array(data)
plt.figure()
plt.plot(data[:, 0], data[:, 1], label="Force (N)")
plt.plot(data[:, 0], data[:, 2], label="Position (m)")
plt.plot(data[:, 0], data[:, 3], label="Angle (rad)")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True)
plt.title("Step Response Data")
plt.show()
