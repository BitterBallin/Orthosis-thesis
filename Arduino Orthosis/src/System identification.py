import serial
import time
import matplotlib.pyplot as plt
from collections import deque
import simpy as sm
import numpy as np
import control as ctl
from scipy.signal import chirp  # To generate the chirp signal
import math

# sm.init_printing()

# Close all open figures
# plt.close('all')

# # === USER SETTINGS ===
# port = 'COM12'
# baudrate = 115200
# max_lines = 2000
# skip_first = 5
# skip_last = 2
# buffer_size = 2000

# # === Setup ===
# ser = serial.Serial(port, baudrate)
# ser.reset_input_buffer()  # Flush old junk from buffer

#=== Motor Identification ===
J = 0.0001  # Moment of inertia of the motor (kg*m^2)
K = 0.01  # Motor constant (N*m/A)
b = 0.1*10**-4  # Damping coefficient (N*m*s/rad)
R = 11.1  # Resistance (Ohm)
L = 0.5*10**-3  # Inductance (H)
V = 0  # Voltage (V) = PWMsignal/255*Vmax


K = 24/(2*math.pi*3500/60)  # Back EMF constant calculated voltage to rad/sec ratio
m = 0.18*0.4 # weight of entire motor and percentage of shaft
r =  0.015
J = 0.5*m*r**2
print("Calculated Rotor Inertia J =", J)

# Define the transfer function:
# Numerator is just [K]
num = [K]

# Denominator comes from expanding (J*s + b)(L*s + R) + K^2:
# That expands to: J*L s^2 + (J*R + b*L)*s + (b*R + K^2)
den = [J*L, (J*R + b*L), (b*R + K**2)]

# Create the transfer function: Angular speed (rad/s) per volt.
motor_tf = ctl.TransferFunction(num, den)
print("Motor Transfer Function (Angular speed/Volt):")
print(motor_tf)


#Defining target position early so it can be used for PID to PWM scaling
targetPosition = 0.08 #80 mm displacement target for full ROM






#Transfer function of wire
#========================================================
# --- Static wire characteristics ---
K = 300000          # [N/m] stiffness of wire
Fi = 200.0              # [N] static load
braid_factor = 1.0     # compensate for braid length
L = 2.0 * braid_factor  # original wire length [m]
Lc = L + Fi / K        # wire length under load [m]
r0 = 0.28e-3           # wire radius [m]

X_current = Lc     # initialize X to static length
rvar = r0          # initialize variable radius

n = 1000 #simulation steps

max_rotations = 500 #max rotations of wire
theta_vec = np.linspace(0, max_rotations*2*math.pi, n)
DX_vec = []

for i, theta in enumerate(theta_vec):
    #Compute amount of rotatios necessary for certain position
    if i == 0:
        X = Lc
        rvar = r0
        DX = 0
        DX_vec.append(DX)
    else:
        rvar = r0*np.sqrt(Lc/X)
        DX = Lc - np.sqrt(Lc**2 - (theta**2) * (rvar**2))
        DX_vec.append(DX)
    if DX > targetPosition:
        break
    

# print(DX_vec)
# print(theta_vec)


# --- polynomial fit ---
deg = 2  # Polynomial degree
coeffs = np.polyfit(theta_vec[0:len(DX_vec)], DX_vec, deg)

# wrap it up into a “callable” poly
DX_poly = np.poly1d(coeffs)

# --- now you can compare or use DX_poly just like compute_DX ---
DX_true = DX_vec
DX_approx = DX_poly(theta_vec[0:len(DX_vec)])
theta_plot = theta_vec[:len(DX_vec)]

# quick printout to see max error
err = np.max(np.abs(DX_true - DX_approx))
print(f"Max abs error over test range: {err:.3e} m")
print("Printing??")

rmse = np.sqrt(np.mean((DX_true - DX_approx)**2))

# --- Plotting ---
plt.figure()
plt.plot(theta_plot, DX_true, label='True DX')
plt.plot(theta_plot, DX_approx, '--', label=f'Poly approx (RMSE={rmse*10**3:.3e} mm, Max abs Er = {err*10**3:.3e} mm)')
plt.xlabel('Theta (rad)')
plt.ylabel('Displacement DX (m)')
plt.title('True vs Polynomial Approximation of Wire Displacement')
plt.legend()
plt.grid(True)
plt.show()

print(DX_poly)

# 1) grab the polynomial coefficients a0…an (highest→lowest power)
a = DX_poly.coeffs
n = DX_poly.order   # degree of the polynomial


num_dx = [DX_poly.coeffs[2], DX_poly.coeffs[1], 2*DX_poly.coeffs[0]]
den_dx = [1, 0, 0, 0]

Wire_tf = ctl.TransferFunction(num_dx, den_dx)
print(Wire_tf)

# Build total Transfer function 
#===============================================

#Integrate motor function so that wire_tf get's angular position instead of speed
# Define an integrator transfer function: 1/s
integrator_tf = ctl.TransferFunction([1], [1, 0])

# Transfer function of spring that is attached to the wire with 1500 n/m
k = 1500
spring_tf = ctl.TransferFunction([1], k)

# Multiply the motor transfer function by 1/s (i.e. integrate)
motor_integrated_tf = motor_tf * integrator_tf





plant_tf = motor_integrated_tf * Wire_tf

# ctl.rootlocus_pid_designer(plant_tf,
#                            gain='P',    # tune Kp (or 'I' or 'D')
#                            Kp0=24/targetPosition,     # your starting guess
#                            Ki0=0.0,
#                            Kd0=0.0,
#                            deltaK=0.01, # how big a step to try each time
#                            plot=True)


# PID controller transfer function
Kp = 24/targetPosition
Ki = 1
Kd = 800

num_pid = [Kd, Kp, Ki]
den_pid = [0, 1, 0]

PID_tf = ctl.TransferFunction(num_pid, den_pid)
print("PID controller Transfer Function:")
print(PID_tf)


# PID OUTPUT RESPONSE 
# PID output simulation: what voltage is sent to the motor?
# Input to PID is the error = reference - output
# Define PID using three proper transfer functions
# tau_d = 0.5  # Small filter time constant for derivative

# Proper PID transfer function
# Define PI part (acting on error)
P_tf = ctl.TransferFunction([Kp], [1])
I_tf = ctl.TransferFunction([Ki], [1, 0])
PI_tf = P_tf + I_tf

# Define D part (acting on output only)
tau_d = 0.01
D_tf = ctl.TransferFunction([Kd, 0], [tau_d, 1])


PID_tf_filter = P_tf + I_tf + D_tf

# Total_tf = plant_tf*PID_tf
Total_tf = plant_tf*PID_tf_filter

print(f"Motor integrated TF {motor_integrated_tf}")
print(f"PID TF {PID_tf_filter}")
print(f"Wire TF {Wire_tf}")
print(f"Spring TF {spring_tf}")
print(f"Plant TF {plant_tf}")

closed_loop_tf = ctl.feedback(Total_tf)


print(f"Total TF {Total_tf}")


# ----- Simulation Setup -----
t_final = 50          # total simulation time in seconds
num_points = 10000     # time steps
t = np.linspace(0, t_final, num_points)

# Open loop chirp
# V_max = 5  # desired max voltage
input_chirp = (chirp(t, f0=0.05, f1=0.5, t1=t_final, method='linear') + 1)/2 * targetPosition
 

# Generate a chirp signal that sweeps from 0.1 Hz to 10 Hz
open_loop_chirp = chirp(t, f0=0.1, f1=10, t1=t_final, method='linear')

# ----- Forced Response for Motor and Total System -----
# For motor_tf (open-loop response)
t_motor, y_motor = ctl.forced_response(motor_tf, T=t, U=open_loop_chirp)

# For Total_tf (motor + PID controller+ wire) - closed-loop response
t_total, y_total = ctl.forced_response(closed_loop_tf, T=t, U=input_chirp)



# Error signal
error_signal = input_chirp - y_total

# Simulate PI on error
_, u_pi = ctl.forced_response(PI_tf, T=t, U=error_signal)

# Simulate D on output
_, u_d = ctl.forced_response(D_tf, T=t, U=y_total)

# Final control signal
u_pid = u_pi - u_d

# ----- Plotting the Chirp Signal and Responses in a 2x1 Grid -----
fig, axs = plt.subplots(2, 1, figsize=(12, 8))

# # Plot 1: Reference vs Motor Output (Open-Loop)
# axs[0].plot(t, input_chirp, label="Reference Chirp")
# axs[0].plot(t_motor, y_motor, '--', label="Motor Output (Open-Loop)")
# axs[0].set_title("Reference vs. Motor Output (Open-Loop)")
# axs[0].set_xlabel("Time [s]")
# axs[0].set_ylabel("Signal")
# axs[0].legend()
# axs[0].grid(True)

# Plot 1: PID Output Voltage
axs[0].plot(t, u_pid, color='orange', label="PID Output Voltage")
axs[0].set_title("PID Output Voltage over Time")
axs[0].set_xlabel("Time [s]")
axs[0].set_ylabel("Voltage [V]")
axs[0].legend()
axs[0].grid(True)

# Plot 2: Reference vs Total System Output (Closed-Loop)
axs[1].plot(t, input_chirp, label="Reference Chirp")
axs[1].plot(t_total, y_total, '--', color='green', label="Total System Output (Closed-Loop)")
axs[1].set_title("Reference vs. Total System Output (Closed-Loop)")
axs[1].set_xlabel("Time [s]")
axs[1].set_ylabel("Signal")
axs[1].legend()
axs[1].grid(True)




plt.tight_layout()
plt.show()

# ===== Plotting Bode Plots in a 2x2 Grid =====
# Define a frequency range for the Bode analysis.
omega_range = np.logspace(-1, 3, 500)  # rad/s

# Get frequency responses using the frequency_response() function.
mag_ol, phase_ol, omega_ol = ctl.frequency_response(motor_tf, omega_range)
mag_cl, phase_cl, omega_cl = ctl.frequency_response(Total_tf, omega_range)

# Convert open-loop magnitude to decibels and phase to degrees.
mag_ol_db = 20 * np.log10(mag_ol)
phase_ol_deg = np.degrees(phase_ol)

# Convert closed-loop magnitude to decibels and phase to degrees.
mag_cl_db = 20 * np.log10(mag_cl)
phase_cl_deg = np.degrees(phase_cl)

# Create a figure with 2x2 subplots.
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Top Left: Open-loop Magnitude
axs[0, 0].semilogx(omega_ol, mag_ol_db, 'b')
axs[0, 0].set_title("Open-Loop Magnitude")
axs[0, 0].set_ylabel("Magnitude (dB)")
axs[0, 0].grid(True)

# Top Right: Open-loop Phase
axs[0, 1].semilogx(omega_ol, phase_ol_deg, 'b')
axs[0, 1].set_title("Open-Loop Phase")
axs[0, 1].set_ylabel("Phase (deg)")
axs[0, 1].grid(True)

# Bottom Left: Closed-loop Magnitude
axs[1, 0].semilogx(omega_cl, mag_cl_db, 'g')
axs[1, 0].set_title("Closed-Loop Magnitude")
axs[1, 0].set_xlabel("Frequency (rad/s)")
axs[1, 0].set_ylabel("Magnitude (dB)")
axs[1, 0].grid(True)

# Bottom Right: Closed-loop Phase
axs[1, 1].semilogx(omega_cl, phase_cl_deg, 'g')
axs[1, 1].set_title("Closed-Loop Phase")
axs[1, 1].set_xlabel("Frequency (rad/s)")
axs[1, 1].set_ylabel("Phase (deg)")
axs[1, 1].grid(True)

plt.tight_layout()
plt.show()
