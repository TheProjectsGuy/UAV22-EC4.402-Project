# Main motion model - North West Up frame, with + configuration
"""
    The code is written for NED frame, X configuration in the file
    `ned_uav_mode.py`. This file is a scratchpad to duplicate it to
    the + configuration, in the NWU frame.
"""


# %% Import everything
import numpy as np
from matplotlib import pyplot as plt
from tqdm import tqdm

# %%
pos_d = np.array([0., 0., 0.])     # Desired X, Y, Z position
ang_d = np.array([0., 0., 0.])     # Desired inertial X, Y, Z angles
vel_d = np.array([0., 0., 0.])     # Desired X, Y, Z velocity
# --- Initial state of the quadrotor ---
pos_init = np.array([0., 0., 0.])  # X, Y, Z   - in m
vel_init = np.array([0., 0., 0.])  # vx, vy, vz - in m/s
ang_init = np.array([0., 0., 0.])  # Phi, Theta, Psi - in rad
# ang_init = np.random.rand(3) / 5.0
ang_vel_init = np.array([0., 0., 0.])  # Phi dot, theta dot, psi dot
# --- Properties of the UAV ---
m = 2.0   # Mass of UAV - in kg
g = 9.8 # Gravity magnitude - in m/(s^2)
J = np.array([[1.2472e-4, 0., 0.],  # Ixx, Ixy, Ixz -|
    [0., 1.2472e-4, 0.],            # Iyx, Iyy, Iyz -- Inertia tensor
    [0., 0., 8.4488e-5]])           # Izx, Izy, Izz -|
pkT = 2e-7  # Thrust constant of propeller (pkT * (n**2) = thrust)
pkt = 1e-9  # Torque constant of propeller (pKt * (n**2) = torque)
"""pkT and pkt use 'n' in RPM (revs per min). SI is rad per sec"""
qL = 0.4    # Distance between propeller centers on the same side (m)
p_max_RPM = 7000.    # Max. revs per minute of the propellers
# --- Controller properties ---
Kp_pos = np.array([0.5, 0.5, 10.0])     # K_p for pos: X, Y, Z
Kd_pos = np.array([1.0, 1.0, 10.0])       # K_d for pos: X, Y, Z
# K_p for inertial ang: X, Y, Z
Kp_ang = np.array([0.3, 0.3, 5.0])
# K_d for inertial ang: X, Y, Z
Kd_ang = np.array([0.1, 0.1, 1.0])
max_phi = np.deg2rad(25.)    # Maximum Phi angle (rot. along X)
max_theta = np.deg2rad(25.)  # Maximum Theta angle (rot. along Y)
max_psi = np.deg2rad(5.)     # Maximum Psi angle (rot. along Z)
# max_ang_acc = 0.5           # Maximum angular 
# --- Simulation properties ---
dt = 5e-4   # Time steps for simulation - in sec
start_time = 0.0    # Start time - in sec
end_time = 15.0     # End time - in sec

# %% Main simulation
# --- Simulation variables ---
time_vals = np.arange(start_time, end_time+dt, dt)
cpos = pos_init # Current position - [x, y, z] in m
cvel = vel_init # Current velocity - [x, y, z] in m/s
gvect = np.array([0., 0., g]) # Gravity in world (inertial) frame
cang = ang_init # Current angles (inertial) - phi, theta, psi
cangvel = ang_vel_init  # Current angular velocity (inertial)
max_angs = np.array([max_phi, max_theta, max_psi])
min_angs = -max_angs    # Minimum bound = -(Maximum bound)
angvel_d = np.array([0., 0., 0.])   # Desired ang. vel.
# --- Logging variables ---
pos_vals = []   # List of x, y, z positions
vel_vals = []   # List of x, y, z velocities
acc_vals = []   # List of x, y, z accelerations
des_acc_vals = []   # List of x, y, z accelerations (desired)
thrustd_vals = []   # List of desired thrust (for UAV propellers)
des_ang_vals = []   # List of phi, theta, psi desired - unclipped
des_ang_clipped_vals = []   # List of Ph, The, Ps desired - clipped
cur_ang_vals = []   # List of current angles - phi, theta, psi
motor_sp_vals = []  # List of motor speeds - n1, n2, n3, n4

# %% Experimental section

# %%
pos_err = pos_d - cpos
vel_err = vel_d - cvel
des_acc = Kp_pos * pos_err + Kd_pos * vel_err
# Compensate for gravity pull (controller needs angle)
des_acc[2] = (des_acc[2] + g)/(np.cos(cang[0])*np.cos(cang[1]))
# Thrust needed (ideal) (in inertial frame) - in +Z axis
des_thrust = m*des_acc[2]   # This is all that the UAV can give!
# Calculate angle desired (from spherical to cartesian formulas)
des_acc_mag = np.linalg.norm(des_acc)
if des_acc_mag == 0:
    des_acc_mag = 1.0   # If no acceleration vector needed
des_ang = np.array([ # Desired angles in the inertial frame
    # Invert 1*sin(phi)*cos(theta) = -acc_y_hat (unit vect.)
    np.arcsin(np.clip(-des_acc[1] / des_acc_mag / np.cos(cang[1]),
        -0.95, 0.95)),    # asin needs only [-1, 1]
    # Invert sin(theta) = acc_x_hat (unit vect.)
    np.arcsin(des_acc[0] / des_acc_mag),
    # Psi is always desired to be zero
    0]
)  # Desired phi, theta, psi calculated
# Threshold angles (cap them)
des_ang_clipped = np.clip(des_ang, min_angs, max_angs)
# Get angle action (angle error -> controller)
ang_err = des_ang_clipped - cang    # Angle error
ang_vel_err = angvel_d - cangvel    # Angular velocity error
des_angacc = Kp_ang * ang_err + Kd_ang * ang_vel_err    # PD act.

# %%

# %%

# %%
