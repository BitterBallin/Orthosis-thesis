

import csv
import numpy as np
import matplotlib.pyplot as plt
from glob import glob

# === Parameters ===
csv_files = glob(r"src\Test Results TB2\Extension Fingertip Test Final\15N\*.csv")
# csv_files = glob(r"src\Test Results TB2\Half Flexion Fingertip Test Final\*.csv")
# csv_files = glob(r"src\Test Results TB2\Full Flexion Test Final\*.csv")

import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import csv

# === Config ===
plot_every_n = 1
clip_time_seconds = 35
regression_degree = 3

# === Define folders with labels and colors ===
test_groups = [
    {
        "label": "Extended",
        "color": "red",
        "files": glob.glob(r"src\Test Results TB2\Extension Fingertip Test Final\15N\*.csv")
    },
    {
        "label": "Half Flexion",
        "color": "blue",
        "files": glob.glob(r"src\Test Results TB2\Half Flexion Fingertip Test Final\*.csv")
    },
    {
        "label": "Full Flexion",
        "color": "#FFB000",
        "files": glob.glob(r"src\Test Results TB2\Full Flexion Test Final\*.csv")
    }
]

plt.figure("Force vs Tip Force with Fit")

# === Process each group ===
for group in test_groups:
    all_force = []
    all_tip_force = []

    for file in group["files"]:
        time_vals = []
        force_vals = []
        tip_force_vals = []

        with open(file, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                if len(row) < 12:
                    continue
                try:
                    time = float(row[0])
                    force = float(row[2])
                    tip_force = float(row[3])
                except ValueError:
                    continue

                time_vals.append(time)
                force_vals.append(force)
                tip_force_vals.append(tip_force)

        # Downsample and clip
        time_vals = time_vals[::plot_every_n]
        force_vals = force_vals[::plot_every_n]
        tip_force_vals = tip_force_vals[::plot_every_n]

        clip_index = next((i for i, t in enumerate(time_vals) if t > clip_time_seconds), len(time_vals))

        force_vals = force_vals[:clip_index]
        tip_force_vals = tip_force_vals[:clip_index]

        all_force.extend(force_vals)
        all_tip_force.extend(tip_force_vals)

    # Polynomial fit
    coeffs = np.polyfit(all_force, all_tip_force, regression_degree)
    predicted = np.polyval(coeffs, all_force)
    r_squared = 1 - np.sum((np.array(all_tip_force) - predicted) ** 2) / np.sum((np.array(all_tip_force) - np.mean(all_tip_force)) ** 2)

    print(f"{group['label']} - R²: {r_squared:.3f}")

    # Plot data points and regression line
    sorted_idx = np.argsort(all_force)
    force_sorted = np.array(all_force)[sorted_idx]
    pred_sorted = np.array(predicted)[sorted_idx]

    plt.plot(all_force, all_tip_force, ',', color=group["color"], markersize=1, alpha=0.5, label=f"{group['label']} Data")
    plt.plot(force_sorted, pred_sorted, '-', color=group["color"], label=f"{group['label']} Fit (R²={r_squared:.3f})")

# === Final plot formatting ===
plt.xlabel("Input Force (N)")
plt.ylabel("Fingertip Force (N)")
plt.title("Input force vs Fingertip force Across Hand Positions")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

plot_every_n = 1  # Downsampling factor

# === Initialize accumulators ===
time_vals_reference = None
force_vals_all = []
tip_force_vals_all = []
position_vals_all = []
target_vals_all = []
p_vals_all = []
i_vals_all = []
d_vals_all = []
control_vals_all = []
rpm_vals_all = []

# === Read and store data from all files ===
for csv_filename in csv_files:
    print(f"🔄 Reading: {csv_filename}")

    time_vals = []
    force_vals = []
    tip_force_vals = []
    position_vals = []
    target_vals = []
    p_vals = []
    i_vals = []
    d_vals = []
    control_vals = []
    rpm_vals = []

    with open(csv_filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header

        for row in reader:
            if len(row) < 12:
                continue  # Ensure complete row
            try:
                time_vals.append(float(row[0]))
                force_vals.append(float(row[2]))
                tip_force_vals.append(float(row[3]))
                position_vals.append(float(row[4]))
                target_vals.append(float(row[5]))
                p_vals.append(float(row[6]))
                i_vals.append(float(row[7]))
                d_vals.append(float(row[8]))
                control_vals.append(float(row[9]))
                rpm_vals.append(float(row[11]))
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

    # === Clip to first 35 seconds ===
    # clip_index = next((i for i, t in enumerate(time_vals) if t > 35), len(time_vals))
    # time_vals       = time_vals[:clip_index]
    # force_vals      = force_vals[:clip_index]
    # tip_force_vals  = tip_force_vals[:clip_index]
    # position_vals   = position_vals[:clip_index]
    # target_vals     = target_vals[:clip_index]
    # p_vals          = p_vals[:clip_index]
    # i_vals          = i_vals[:clip_index]
    # d_vals          = d_vals[:clip_index]
    # control_vals    = control_vals[:clip_index]
    # rpm_vals        = rpm_vals[:clip_index]

    # Set reference time on first file
    if time_vals_reference is None:
        time_vals_reference = time_vals

    # Append clipped signals
    force_vals_all.append(force_vals)
    tip_force_vals_all.append(tip_force_vals)
    position_vals_all.append(position_vals)
    target_vals_all.append(target_vals)
    p_vals_all.append(p_vals)
    i_vals_all.append(i_vals)
    d_vals_all.append(d_vals)
    control_vals_all.append(control_vals)
    rpm_vals_all.append(rpm_vals)

# Finally, truncate time reference to shortest
min_len = min(len(lst) for lst in position_vals_all)
time_vals_reference = time_vals_reference[:min_len]


# === Utilities ===
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
import numpy as np

# Step 1: Compute RMSE per test
rmse_list = []
for tip_force_vals, target_vals in zip(tip_force_vals_all, target_vals_all):
    min_len = min(len(tip_force_vals), len(target_vals))
    tip_force_vals_trunc = tip_force_vals[:min_len]
    target_vals_trunc = target_vals[:min_len]
    
    squared_errors = [(p - t) ** 2 for p, t in zip(tip_force_vals_trunc, target_vals_trunc)]
    rmse = np.sqrt(np.mean(squared_errors))
    rmse_list.append(rmse)

# Step 2: Compute mean and variance of RMSEs
mean_rmse = np.mean(rmse_list)
rmse_variance = np.var(rmse_list)

# Step 3: Normalize the variance to the RMSE
normalized_variance = rmse_variance / (mean_rmse ** 2)

print("Mean RMSE:", mean_rmse)
print("RMSE Variance:", rmse_variance)
print("Normalized Variance:", normalized_variance)

print("RMSE variance across tests:", rmse_variance)

# === Compute averages ===
force_vals = average_lists(truncate_to_shortest(force_vals_all))
tip_force_vals = average_lists(truncate_to_shortest(tip_force_vals_all))
position_vals = average_lists(truncate_to_shortest(position_vals_all))
target_vals = average_lists(truncate_to_shortest(target_vals_all))
p_vals = average_lists(truncate_to_shortest(p_vals_all))
i_vals = average_lists(truncate_to_shortest(i_vals_all))
d_vals = average_lists(truncate_to_shortest(d_vals_all))
control_vals = average_lists(truncate_to_shortest(control_vals_all))
rpm_vals = average_lists(truncate_to_shortest(rpm_vals_all))

# === RMSE calculation ===
squared_errors = [(p - t) ** 2 for p, t in zip(tip_force_vals, target_vals)]
rmse = np.sqrt(np.mean(squared_errors))

# === Linear regression (Force ➜ Tip Force) & R² ===
coeffs        = np.polyfit(force_vals, tip_force_vals, 3)  # slope, intercept
# slope, offset = coeffs
predicted     = np.polyval(coeffs, force_vals)
ss_res        = np.sum((tip_force_vals - predicted) ** 2)
ss_tot        = np.sum((tip_force_vals - np.mean(tip_force_vals)) ** 2)
r_squared     = 1 - ss_res / ss_tot
print(f"Coefficient of determination (R²): {r_squared:.3f}")


# === Plotting ===
plt.close('all')

plt.figure("Position vs Target with control signal")
plt.plot(time_vals_reference, tip_force_vals, label='Fingertip Force (N))')
plt.plot(time_vals_reference, target_vals, label='Force Target (N)', linestyle='--')
plt.plot(time_vals_reference, [c / 100 for c in control_vals], label='Control Signal (scaled)', linestyle='-.')
plt.axhline(25.5, color='gray', linestyle='--', linewidth=1, label = 'Max Control signal [0,25]')
plt.axhline(-25.5, color='gray', linestyle='--', linewidth=1)

plt.xlabel("Time (s)")
plt.ylabel("Scaled Values")
plt.title(f"Control Signal, Position & Target (RMSE = {rmse:.2f} N)")
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

plt.figure("Force Over Time")
plt.plot(time_vals_reference, force_vals, 'b-')
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Force Over Time")
plt.grid(True)

plt.figure("Tip Force Over Time")
plt.plot(time_vals_reference, tip_force_vals, 'r-')
plt.xlabel("Time (s)")
plt.ylabel("Tip Force (N)")
plt.title("Tip Force Over Time")
plt.grid(True)

plt.figure("Force vs Tip Force with Fit")
# scatter
plt.plot(force_vals, tip_force_vals, "b.", markersize = 1, label="Data")
# regression line (sorted)
sorted_idx   = np.argsort(force_vals)
plt.plot(np.array(force_vals)[sorted_idx],
         predicted[sorted_idx],
         'r-', label=f'Polynomial Fit (degree {len(coeffs)-1})')
plt.xlabel("Input Force (N)")
plt.ylabel("Tip Force (N)")
plt.title("Input Force vs Tip Force")
plt.legend()
plt.grid(True)

plt.figure("RPM Over Time")
plt.plot(time_vals_reference, rpm_vals, label='RPM', color='green')
plt.xlabel("Time (s)")
plt.ylabel("RPM")
plt.title("RPM vs Time")
plt.grid(True)

plt.show()
