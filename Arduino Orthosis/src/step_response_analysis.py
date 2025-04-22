import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import control as ctl
from scipy.interpolate import interp1d

# === USER SETTINGS ===
PORT = 'COM12'
BAUDRATE = 115200
MAX_LINES = 2000
SKIP_FIRST = 5
SKIP_LAST = 2

# === Serial Setup ===
print("Connecting to serial...")
ser = serial.Serial(PORT, BAUDRATE, timeout=2)
ser.reset_input_buffer()
print("Reading serial...")

# === Read Data from Serial ===
lines = []
while len(lines) < MAX_LINES:
    if ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if "END" in line:
                break
            if "," in line:
                lines.append(line)
                if len(lines) % 100 == 0:
                    print(f"Read {len(lines)} lines...")
        except UnicodeDecodeError:
            continue  # skip garbage

ser.close()
print(f"Total lines read: {len(lines)}")

# === Parse Data ===
data = []
for line in lines:
    try:
        t_str, y_str = line.split(",")
        t = float(t_str)
        y = float(y_str)
        data.append([t, y])
    except ValueError:
        continue

data = np.array(data)

# === Validate and Trim ===
if data.shape[0] < 10:
    raise RuntimeError("Not enough data received to perform analysis.")

t = data[:, 0]
y = data[:, 1]

if len(t) > SKIP_FIRST + SKIP_LAST:
    t = t[SKIP_FIRST:-SKIP_LAST]
    y = y[SKIP_FIRST:-SKIP_LAST]

# Normalize time
t -= t[0]

# === Resample to uniform time vector ===
t_uniform = np.linspace(t[0], t[-1], len(t))
interp_func = interp1d(t, y, kind='linear')
y_uniform = interp_func(t_uniform)

# === Polynomial Fit ===
deg = 2
coeffs = np.polyfit(t_uniform, y_uniform, deg)
y_poly = np.poly1d(coeffs)
y_fit = y_poly(t_uniform)

# === Error metrics ===
err_max = np.max(np.abs(y_uniform - y_fit))
rmse = np.sqrt(np.mean((y_uniform - y_fit) ** 2))
print(f"Max abs error: {err_max:.4e}")
print(f"RMSE:          {rmse:.4e}")

# === Transfer Function Derivation from 2nd order poly ===
a = coeffs  # [a2, a1, a0]
num = [a[2], a[1], 2*a[0]]
den = [1, 0, 0, 0]  # s^3 denominator

step_tf = ctl.TransferFunction(num, den)
print("\nDerived Transfer Function from Polynomial Fit:")
print(step_tf)

# === Plot ===
plt.figure()
plt.plot(t_uniform, y_uniform, label='Measured (uniform)')
plt.plot(t_uniform, y_fit, 'r--', label=f'Poly Fit (deg={deg})')
plt.xlabel('Time [s]')
plt.ylabel('Output [radians]')
plt.title('Step Response and Polynomial Fit')
plt.legend()
plt.grid(True)
plt.show()



# === Input vs Output Plot ===
step_input = np.ones_like(t_uniform)  # Step of magnitude 1
step_input *= y_uniform[-1]  # Scale to match final value of the output (gain)

plt.figure()
plt.plot(t_uniform, step_input, 'k--', label='Input (Step)')
plt.plot(t_uniform, y_uniform, label='Output (Measured)')
plt.plot(t_uniform, y_fit, 'r--', label=f'Poly Fit (deg={deg})')
plt.xlabel('Time [s]')
plt.ylabel('Signal')
plt.title('Step Input vs System Output')
plt.grid(True)
plt.legend()
plt.show()


# === Compute Angular Velocity ===
dt = np.diff(t_uniform)
dy = np.diff(y_uniform)
omega = dy / dt  # rad/s

# Make time vector for omega same length
t_omega = t_uniform[1:]

# Create matching step input vector for omega plot
step_input_vel = np.ones_like(omega) * 3500*2*np.pi/60  # scaled to match output gain

# === Plot Angular Velocity vs Input ===
plt.figure()
plt.plot(t_omega, omega, label='Angular Velocity (rad/s)')
plt.plot(t_omega, step_input_vel, 'k--', label='Input (Step)')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.title('Angular Velocity vs Step Input')
plt.grid(True)
plt.legend()
plt.show()
