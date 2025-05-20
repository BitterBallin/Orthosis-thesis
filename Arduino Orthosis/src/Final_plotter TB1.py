import csv
import numpy as np
import matplotlib.pyplot as plt
from glob import glob

# === Parameters ===
# csv_files = glob(r"src\Test results TB1\TB1_RAMP_TEST_SPRING1_SLOOP_24V_140CM_2025-05-06.csv")
# csv_files = glob(r"src/Test Results TB1/Spring 2 24V CHIRP tests/*.csv")
# csv_files = glob(r"src/Test Results TB1/Spring 2 24V Ramp test/*.csv")
# csv_files = glob(r"src/Test Results TB1/Spring 2 30 V Ramp test/*.csv")
# csv_files = glob(r"src/Test Results TB1/TB1_Ramp_test_24V_80CM_1_2025-05-01.csv")
# csv_files = glob(r"src/Test Results TB1/TB1_Ramp_test_30V1_170CM_2025-05-01.csv")
csv_files = glob(r"src/Test Results TB1/TB1_CHIRPTEST_TEST1_30V_80CM_2025-05-01.csv")

# csv_files = glob(r"src/Test Results TB2/Two Sided Test Final/*.csv")


plot_every_n = 1  # Downsampling factor

# === Initialize accumulators ===
time_vals_reference = None
position_vals_all = []
target_vals_all = []
p_vals_all = []
i_vals_all = []
d_vals_all = []
control_vals_all = []
rpm_vals_all = []
force_vals_all = []

# === Read and store data from all files ===
for csv_filename in csv_files:
    print(f"🔄 Reading: {csv_filename}")

    time_vals = []
    position_vals = []
    target_vals = []
    p_vals = []
    i_vals = []
    d_vals = []
    control_vals = []
    rpm_vals = []
    force_vals = []

    with open(csv_filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header

        for row in reader:
            if len(row) < 11:
                continue  # Ensure complete row
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

    # Downsample
    time_vals = time_vals[::plot_every_n]
    force_vals = force_vals[::plot_every_n]
    position_vals = position_vals[::plot_every_n]
    target_vals = target_vals[::plot_every_n]
    p_vals = p_vals[::plot_every_n]
    i_vals = i_vals[::plot_every_n]
    d_vals = d_vals[::plot_every_n]
    control_vals = control_vals[::plot_every_n]
    rpm_vals = rpm_vals[::plot_every_n]

    # Set time reference once
    if time_vals_reference is None:
        time_vals_reference = time_vals

    # Append all signals
    force_vals_all.append(force_vals)
    position_vals_all.append(position_vals)
    target_vals_all.append(target_vals)
    p_vals_all.append(p_vals)
    i_vals_all.append(i_vals)
    d_vals_all.append(d_vals)
    control_vals_all.append(control_vals)
    rpm_vals_all.append(rpm_vals)

    min_len = min(len(lst) for lst in position_vals_all)
    time_vals_reference = time_vals_reference[:min_len]

# === Average utility ===
def average_lists(list_of_lists):
    lens = list(map(len, list_of_lists))
    if len(set(lens)) > 1:
        print(f"⚠️ Inconsistent lengths: {lens}")
        min_len = min(lens)
        list_of_lists = [lst[:min_len] for lst in list_of_lists]
    arr = np.array(list_of_lists)
    return np.mean(arr, axis=0).tolist()

def truncate_to_shortest(list_of_lists):
    min_len = min(len(lst) for lst in list_of_lists)
    return [lst[:min_len] for lst in list_of_lists]

# === Compute averages ===
force_vals = average_lists(truncate_to_shortest(force_vals_all))
position_vals = average_lists(truncate_to_shortest(position_vals_all))
target_vals = average_lists(truncate_to_shortest(target_vals_all))
p_vals = average_lists(truncate_to_shortest(p_vals_all))
i_vals = average_lists(truncate_to_shortest(i_vals_all))
d_vals = average_lists(truncate_to_shortest(d_vals_all))
control_vals = average_lists(truncate_to_shortest(control_vals_all))
rpm_vals = average_lists(truncate_to_shortest(rpm_vals_all))

# === RMSE calculation ===
squared_errors = [(p - t) ** 2 for p, t in zip(position_vals, target_vals)]
rmse = np.sqrt(np.mean(squared_errors))

# === Plotting ===
plt.close('all')

plt.figure("Position vs Target with control signal")
plt.plot(time_vals_reference, [p * 1000 for p in position_vals], label='Position (mm)')
plt.plot(time_vals_reference, [t * 1000 for t in target_vals], label='Target (mm)', linestyle='--')
plt.plot(time_vals_reference, [c / 10 for c in control_vals], label='Control Signal (scaled)', linestyle='-.')

plt.axhline(25.5, color='gray', linestyle='--', linewidth=1,  label = 'Max PWM control signal [0-25]')
plt.axhline(-25.5, color='gray', linestyle='--', linewidth=1)

plt.xlabel("Time (s)")
plt.ylabel("Scaled Values")
plt.title(f"Control Signal, Position & Target (RMSE = {rmse * 1000:.2f} mm)")
plt.legend()
plt.grid(True)

plt.figure("PID Terms")
plt.plot(time_vals_reference, p_vals, label='P')
plt.plot(time_vals_reference, i_vals, label='I')
plt.plot(time_vals_reference, d_vals, label='D')
plt.xlabel("Time (s)")
plt.ylabel("PID Contributions")
plt.title("P, I, D Terms Over Time")
plt.legend()
plt.grid(True)

plt.figure("RPM Over Time")
plt.plot(time_vals_reference, rpm_vals, label='RPM', color='green')
plt.xlabel("Time (s)")
plt.ylabel("RPM")
plt.title("RPM vs Time")
plt.grid(True)

plt.figure("Force Over Time")
plt.plot(time_vals_reference, force_vals, label='Force (N)', color='red')
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Force vs Time")
plt.grid(True)

plt.show()
