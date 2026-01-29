# -*- coding: utf-8 -*-
"""
Company Project: Asset management and testing – Mechanical

Date: 28-01-2025

Created by: Maikel van der Linden & Dennis Kaale
Modified by: Max De Muirier (HAW Hamburg)
"""

# import libraries for various operations
import numpy as np
import matplotlib.pyplot as plt
from IPython import get_ipython
import math 
get_ipython().run_line_magic('matplotlib', 'qt')



'''
SIMULATION SETTINGS
'''

# Final position where the arm needs to move
end_X = 385                            # X-coordinate of the final position
end_Y = 0                              # Y-coordinate of the final position
end_Z = 330                            # Z-coordinate of the final position

step_size = 5                          # Step size for the arrange function

d = {
     # DIMENSIONS FROM SOLIDWORKS DESIGN
     
     # Lengths of the arm
     'L_0_A1':45,          # Length of arm 0_A1 (between 0 and A1) [mm]45
     'L_A1_A':25,        # Length of arm A1_A (between A1 and A) [mm]25
     'L_A_B':120,         # Length of arm A_B (between A and B) [mm]
     'L_B_C':137.71,       # Length of arm B_C (between B and C) [mm]
     'L_C_D':160,          # Length of arm C_D (between C and D) [mm]
     
     
     # CYLINDER A - dimensions between points
     'L_A_a1':50.7,       # Length between point A and a1 [mm]
     'L_a1_a2':11,      # Length between point b1 and b2 [mm]     
     'L_A_a4':46.04,      # Length between point A and a3 [mm]
     'L_a3_a4':-1.3,       # Length between point a2 and a3 [mm]
    
     # CYLINDER A - cylinder properties
     'C_A_min':66,         # Cylinder A minimum length [mm]
     'C_A_stroke':36,      # Cylinder A stroke [mm]
     "C_A_lim": 8.5,       # Filling cylinder A [mm]
     'C_A_dia_piston': 9.5,# diameter piston cylinder A [mm]
     'C_A_dia_rod': 4,     # diameter rod cylinder A [mm]
     
     
     # CYLINDER B - dimensions between points
     'L_B_b1':62.92,       # Length between point B and b1 [mm]
     'L_b1_b2':45,      # Length between point b1 and b2 [mm]
     'L_B_b4':52.3,        # Length between point B and b4 [mm]
     'L_b3_b4':-1.09,      # Length between point b3 and b4 [mm]
     
     # CYLINDER B - cylinder properties
     'C_B_min':75,         # Cylinder B minimum length [mm]
     'C_B_stroke':45,      # Cylinder B stroke [mm]
     'C_B_dia_piston': 9.5,# diameter piston cylinder B
     'C_B_dia_rod': 4,     # diameter rod cylinder B [mm]
     
     
     # CYLINDER C - dimensions between points
     'L_C_c1':104.98,      # Length between point C and c2 [mm]
     'L_c1_c2':21.09,      # Length between point c1 and c2 [mm]
     'L_C_c4':30,          # Length between point C and c4 [mm]
     'L_c3_c4':21.09,      # Length between point c3 and c4 [mm]
     
     # CYLINDER C - cylinder properties
     'C_C_min':80,         # Cylinder C minimum length [mm]
     'C_C_stroke':50,      # Cylinder C stroke [mm] 
     'C_C_dia_piston': 9.5,# diameter piston cylinder C [mm]
     'C_C_dia_rod': 4,     # diameter rod cylinder C [mm]  
     
     
     # OTHERS
     'Angle_base': 360,    # Angle that the base can rotate [deg]
     'max_error': 1,       # Maximum allowable error in final coordinates [mm] 
     }


'''
CONSTANTS THAT NEED TO BE CALCULATED ONCE
'''

# CYLINDER A - dimensions between points
L_A_a2 = np.sqrt((d['L_a1_a2']**2) +(d['L_A_a1']**2))       # Length between point B and b2 [mm]
L_A_a3 = np.sqrt((d['L_a3_a4']**2) + (d['L_A_a4']**2))    # Length between point A and a2 [mm] 

# CYLINDER A - cylinder properties
C_A_max = d['C_A_min'] + d['C_A_stroke'] - d['C_A_lim']     # Maximum cylinder length for cylinder A
alpha_A1 = np.degrees(np.arctan(d['L_a1_a2']/d['L_A_a1']))  # Angle alpha_B1  
alpha_A3 = np.degrees(np.arctan(d['L_a3_a4']/d['L_A_a4']))  # Angle alpha_B3
alpha_A1_rad = np.deg2rad(alpha_A1)                         # Angle alpha_B1 radians
alpha_A3_rad = np.deg2rad(alpha_A3)                         # Angle alpha_B3 radians

# CYLINDER A - Minimum and maximum angle at point A
Alpha_A_min = np.arccos((L_A_a2**2 + L_A_a3**2 - d['C_A_min']**2 ) / 
    (2*L_A_a2*L_A_a3)) + alpha_A1_rad + alpha_A3_rad                 # Minimum angle A
Alpha_A_max = np.arccos((L_A_a2**2 + L_A_a3**2 - C_A_max**2 ) / 
    (2*L_A_a2*L_A_a3)) + alpha_A1_rad + alpha_A3_rad                  # Maximum angle A
Alpha_A_avg = (Alpha_A_min + Alpha_A_max)/2

# CYLINDER B - dimensions between points
L_B_b2 = np.sqrt((d['L_b1_b2']**2) +(d['L_B_b1']**2))       # Length between point B and b2 [mm]
L_B_b3 = np.sqrt((d['L_b3_b4']**2) + (d['L_B_b4']**2))      # Length between point B and b3 [mm]

# CYLINDER B - cylinder properties
C_B_max = d['C_B_min'] + d['C_B_stroke']                    # Maximum cylinder length for cylinder B
alpha_B1 = np.degrees(np.arctan(d['L_b1_b2']/d['L_B_b1']))  # Angle alpha_B1  
alpha_B3 = np.degrees(np.arctan(d['L_b3_b4']/d['L_B_b4']))  # Angle alpha_B3
alpha_B1_rad = np.deg2rad(alpha_B1)                         # Angle alpha_B1 radians
alpha_B3_rad = np.deg2rad(alpha_B3)                         # Angle alpha_B3 radians

# CYLINDER B - Minimum and maximum angle at point B
Alpha_B_min = (np.arccos(((L_B_b2**2) + (L_B_b3**2) - (d['C_B_min']**2)) / 
    (2*L_B_b2*L_B_b3)) + alpha_B1_rad + alpha_B3_rad) - np.pi # Minimum angle B
Alpha_B_max = (np.arccos(((L_B_b2**2) + (L_B_b3**2) - (C_B_max**2)) / 
    (2*L_B_b2*L_B_b3)) + alpha_B1_rad + alpha_B3_rad) - np.pi # Maximum angle B
Alpha_B_avg = (Alpha_B_min + Alpha_B_max)/2



# CYLINDER C - dimensions between points
L_C_c2 = np.sqrt((d['L_c1_c2']**2) +(d['L_C_c1']**2))       # Length between point C and c2 [mm]
L_C_c3 = np.sqrt((d['L_c3_c4']**2) + (d['L_C_c4']**2))      # Length between point C and c3 [mm]

# CYLINDER C - cylinder properties 
C_C_max = d['C_C_min'] + d['C_C_stroke']                      # Cylinder C max [mm]
alpha_C1 = np.degrees(np.arctan(d['L_c1_c2']/d['L_C_c1']))  # Angle alpha_C1  
alpha_C3 = np.degrees(np.arctan(d['L_c3_c4']/d['L_C_c4']))  # Angle alpha_C3
alpha_C1_rad = np.deg2rad(alpha_C1)
alpha_C3_rad = np.deg2rad(alpha_C3)

# CYLINDER C - Minimum and maximum angle at point C
Alpha_C_min = 2*np.pi - (np.arccos(((L_C_c2**2) + (L_C_c3**2) - (d['C_C_min']**2)) / 
    (2*L_C_c2*L_C_c3)) + alpha_C1_rad + alpha_C3_rad)-np.pi # Minimum angle C
Alpha_C_max = 2*np.pi - (np.arccos(((L_C_c2**2) + (L_C_c3**2) - (C_C_max**2)) / 
    (2*L_C_c2*L_C_c3)) + alpha_C1_rad + alpha_C3_rad)-np.pi # Maximum angle C
Alpha_C_avg = (Alpha_C_min + Alpha_C_max)/2



# DETERMINE FINAL POSITION AT AVERAGE ANGLES
XY_AVG_B = d['L_A1_A']+(-np.cos(Alpha_A_avg)*d['L_A_B'])
Z_AVG_B = d['L_0_A1']+(np.sin(Alpha_A_avg)*d['L_A_B'])

XY_AVG_C = XY_AVG_B + (-np.cos(Alpha_A_avg+Alpha_B_avg)*d['L_B_C'])
Z_AVG_C = Z_AVG_B + (np.sin(Alpha_A_avg+Alpha_B_avg)*d['L_B_C'])

XY_AVG_D = XY_AVG_C + (-np.cos(Alpha_A_avg+Alpha_B_avg+Alpha_C_avg)*d['L_C_D'])
Z_AVG_D = Z_AVG_C + (np.sin(Alpha_A_avg+Alpha_B_avg+Alpha_C_avg)*d['L_C_D'])

end_alpha_avg = np.arctan((Z_AVG_D - Z_AVG_C) / (XY_AVG_D - XY_AVG_C))
end_alpha_avg_deg = np.rad2deg(end_alpha_avg)

if end_X<0 or end_Y<0:
    end_XY = -np.sqrt(end_X**2 + end_Y**2)
else:
    end_XY = np.sqrt(end_X**2 + end_Y**2)
    
'''
CREATE ANIMATION AND DETERMINE RANGE OF FINAL POSITION
'''

ns = 300            # Number of steps for range
n_phase = ns // 6    # Divide the total number of steps over 6 phases

# # CREATE A LINSPACE FOR EXTENDING ALL CYLINDERS
C_A_range = np.concatenate([
    np.linspace(d['C_A_min'], C_A_max, n_phase), # Phase 1: Cylinder A extends
    np.full(n_phase, C_A_max),                   # Phase 2: Cylinder A remains at max
    np.full(n_phase, C_A_max),                   # Phase 3: Cylinder A remains at max
    np.linspace(C_A_max, d['C_A_min'], n_phase), # Phase 4: Cylinder A retracts
    np.full(n_phase, d['C_A_min']),              # Phase 5: Cylinder A remains at min
    np.full(n_phase, d['C_A_min'])               # Phase 6: Cylinder A remains at min
])

C_B_range = np.concatenate([
    np.full(n_phase, d['C_B_min']),              # Phase 1: Cylinder B remains at min
    np.linspace(d['C_B_min'], C_B_max, n_phase), # Phase 2: Cylinder B extends
    np.full(n_phase, C_B_max),                   # Phase 3: Cylinder B remains at max
    np.full(n_phase, C_B_max),                   # Phase 4: Cylinder B remains at max
    np.linspace(C_B_max, d['C_B_min'], n_phase), # Phase 5: Cylinder B retracts
    np.full(n_phase, d['C_B_min'])               # Phase 6: Cylinder B remains at min
])

C_C_range = np.concatenate([
    np.full(n_phase,  C_C_max),                  # Phase 1: Cylinder C remains at min
    np.full(n_phase,  C_C_max),                  # Phase 2: Cylinder C remains at min
    np.linspace( C_C_max, d['C_C_min'], n_phase),# Phase 3: Cylinder C extends
    np.full(n_phase, d['C_C_min']),              # Phase 4: Cylinder C remains at max
    np.full(n_phase, d['C_C_min']),              # Phase 5: Cylinder C remains at max
    np.linspace(d['C_C_min'], C_C_max, n_phase)  # Phase 6: Cylinder C retracts
])


# CYLINDER A - rotation at point A
alpha_A2_range = np.degrees(
    np.arccos((L_A_a2**2 + L_A_a3**2 - C_A_range**2 ) / 
    (2*L_A_a2*L_A_a3))) # Angle range alpha_A2


# CYLINDER B - rotation at point B
alpha_B2_range = np.degrees(
    np.arccos(((L_B_b2**2) + (L_B_b3**2) - (C_B_range**2)) / 
    (2*L_B_b2*L_B_b3))) # Angle range alpha_B2


# CYLINDER C - rotation at point C
alpha_C2_range = np.degrees(
    np.arccos(((L_C_c2**2) + (L_C_c3**2) - (C_C_range**2)) / 
    (2*L_C_c2*L_C_c3))) # Angle range alpha_C2


# CALCULATIONS OF ANGLES BETWEEN ARMS
alpha_A1_A2_A3_range = np.radians(alpha_A1 + alpha_A2_range + alpha_A3)                                  # Total angle between the base plane of the drone and L_A_B
alpha_B1_B2_B3_range = np.radians(alpha_B1 + alpha_B2_range + alpha_B3)                    # Total angle between L_A_B and L_B_C
alpha_C1_C2_C3_range = np.radians(alpha_C1 + alpha_C2_range + alpha_C3)                 # Total angle between L_B_C and L_C_D


# CALCULATIONS OF COORDINATES OF PIVOT POINTS
XY_B = d['L_A1_A'] - np.cos(alpha_A1_A2_A3_range) * d['L_A_B']
Z_B = d['L_0_A1'] + np.sin(alpha_A1_A2_A3_range) * d['L_A_B']

XY_C = XY_B - np.sin(alpha_B1_B2_B3_range - (np.pi/2 - alpha_A1_A2_A3_range)) * d['L_B_C']
Z_C = Z_B - np.cos(alpha_B1_B2_B3_range - (np.pi/2 - alpha_A1_A2_A3_range)) * d['L_B_C']

XY_D = XY_C - np.sin(alpha_C1_C2_C3_range - (np.pi/2 - (alpha_B1_B2_B3_range - (np.pi/2 - alpha_A1_A2_A3_range)))) * d['L_C_D']
Z_D = Z_C - np.cos(alpha_C1_C2_C3_range - (np.pi/2 - (alpha_B1_B2_B3_range - (np.pi/2 - alpha_A1_A2_A3_range)))) * d['L_C_D']


# Plot Position in the xy-z plane
plt.figure()
plt.title('Position of pivot points')
plt.scatter(d['L_A1_A'], d['L_0_A1'], color='gray')
# plt.scatter(XY_AVG_B, Z_AVG_B, color='blue')
# plt.scatter(XY_AVG_C, Z_AVG_C, color='orange')
# plt.scatter(XY_AVG_D, Z_AVG_D, color='green', label= 'End position with average angles')
# plt.scatter(end_XY, end_Z, color='red', label= 'Input end position')
plt.plot(XY_B, Z_B, label='Angular range B')
plt.plot(XY_C, Z_C, label='Angular range C')
plt.plot(XY_D, Z_D, label='Angular range D')
plt.xlabel('X [mm]')
plt.ylabel('Z [mm]')
plt.legend()
plt.xlim(-250, 400)
plt.ylim(-50, 450)
plt.grid()
plt.show()

# Plot of cylinder motion cycle
plt.figure()
plt.title('Cycle of motion of cylinders')
plt.plot(C_A_range, label='CYLINDER A (C_A)')
plt.plot(C_B_range, label='CYLINDER B (C_B)')
plt.plot(C_C_range, label='CYLINDER C (C_C)')
plt.xlabel('Step [-]')
plt.ylabel('Cylinder length [mm]')
plt.legend()
plt.grid()
plt.show()

# Animation of robot arm movement over the maximum range
plt.figure()
plt.title('Animation of robot arm movement')
arm_plot, = plt.plot([], [], 'o-', label='Robot arm')
plt.xlabel('x [mm]')
plt.ylabel('y [mm]')
plt.xlim(min(XY_B.min(), XY_C.min(), XY_D.min()) - 50, 
          max(XY_B.max(), XY_C.max(), XY_D.max()) + 50)
plt.ylim(min(Z_B.min(), Z_C.min(), Z_D.min()) - 50, 
          max(Z_B.max(), XY_C.max(), Z_B.max()) + 50)
plt.xlim(-250, 400)
plt.ylim(-50, 450)
plt.grid()
plt.legend()

# Animation loop
for i in range(len(C_A_range)):   
    # Update the positions of the pivot points
    x_coords = [d['L_A1_A'], XY_B[i], XY_C[i], XY_D[i]]
    z_coords = [d['L_0_A1'], Z_B[i], Z_C[i], Z_D[i]]
    arm_plot.set_data(x_coords, z_coords)

    # Update title
    plt.title(f'Animation of robot arm movement - Step {i+1}/{len(C_A_range)}')

    # Show and pause for animation effect
    plt.draw()
    plt.pause(0.01)

# Keep the window open after the animation
plt.show()



'''
NUMERICAL CALCULATION OF ROBOT ARM MOVEMENT BY CYLINDER ACTUATION
'''

alpha_Base = np.arange(0,d['Angle_base'],step_size)    # Base angle range
C_A = np.arange(d['C_A_min'],C_A_max,step_size)       # Range of cylinder A
C_B = np.arange(d['C_B_min'],C_B_max,step_size)       # Range of cylinder B
C_C = np.arange(d['C_C_min'],C_C_max,step_size)       # Range of cylinder C


# CYLINDER A - rotation at point A
alpha_A2 = np.degrees(
    np.arccos((L_A_a2**2 + L_A_a3**2 - C_A**2 ) / 
    (2*L_A_a2*L_A_a3)))          # Angle alpha_A2


# CYLINDER B - rotation at point B
alpha_B2 = np.degrees(
    np.arccos(((L_B_b2**2) + (L_B_b3**2) - (C_B**2)) / 
    (2*L_B_b2*L_B_b3)))                 # Angle alpha_B2


# CYLINDER C - rotation at point C
alpha_C2 = np.degrees(
    np.arccos(((L_C_c2**2) + (L_C_c3**2) - (C_C**2)) / 
    (2*L_C_c2*L_C_c3)))                 # Angle alpha_C2



# CALCULATIONS OF ANGLES BETWEEN ARMS
alpha_A1_A2_A3 = np.radians(alpha_A1 + alpha_A2 + alpha_A3)                 # Total angle between the drone base and L_A_B
alpha_B1_B2_B3 = np.radians(alpha_B1 + alpha_B2 + alpha_B3)                 # Total angle between L_A_B and L_B_C
alpha_C1_C2_C3 = np.radians(alpha_C1 + alpha_C2 + alpha_C3)                 # Total angle between L_B_C and L_C_D


# CALCULATION OF ANGLES BETWEEN ARMS RELATIVE TO ZERO
alpha_A_res = np.rad2deg(alpha_A1_A2_A3)
alpha_B_res = np.rad2deg(alpha_B1_B2_B3 - np.pi)
alpha_C_res = np.rad2deg(2*np.pi - alpha_C1_C2_C3 - np.pi)

alpha_Base_res_rad = np.deg2rad(alpha_Base)
alpha_A_res_rad = np.deg2rad(alpha_A1_A2_A3)
alpha_B_res_rad = np.deg2rad(alpha_B1_B2_B3 - 180)
alpha_C_res_rad = np.deg2rad(360 - alpha_C1_C2_C3 - 180)


# CREATE GRID POINTS FOR COORDINATE CONTROL WITH RESULTANT ANGLES
[ALPHA_BASE, ALPHA_A, ALPHA_B, ALPHA_C] = np.meshgrid(alpha_Base_res_rad,alpha_A_res_rad,alpha_B_res_rad,alpha_C_res_rad) 



'''
DEFINE FUNCTIONS FOR INVERSE KINEMATICS
'''


# CALCULATIONS OF COORDINATES OF PIVOT POINTS
def an_kin(ALPHA_A, ALPHA_B, ALPHA_C, ALPHA_BASE):
    xy_B = -np.cos(ALPHA_A) * d['L_A_B'] + d['L_A1_A']               # Position segment A + B XY-plane
    xy_C = xy_B + (-np.cos(ALPHA_A + ALPHA_B) * d['L_B_C'])          # Position segment B + C XY-plane
    xy_D = xy_C + (-np.cos(ALPHA_A + ALPHA_B + ALPHA_C) * d['L_C_D'])  # Position segment C + D XY-plane
    X = xy_D * np.cos(ALPHA_BASE)                                   # Final position segment D X-plane
    Y = xy_D * np.sin(ALPHA_BASE)                                   # Final position segment D Y-plane

    z_B = np.sin(ALPHA_A) * d['L_A_B'] + d['L_0_A1']                # Position segment A + B Z-plane
    z_C = z_B + np.sin(ALPHA_A + ALPHA_B) * d['L_B_C']              # Position segment B + C Z-plane
    z_D = z_C + np.sin(ALPHA_A + ALPHA_B + ALPHA_C) * d['L_C_D']    # Position segment C + D Z-plane
    Z = z_D                                                         # Final position segment D Z-plane
    end_alpha = ALPHA_A + ALPHA_B + ALPHA_C                         # Angle of the last segment (customizable)

    XYZ = np.array([[0,0,0],                                    # Starting position
            [d['L_A1_A'] *np.cos(ALPHA_BASE),d['L_A1_A'] *np.sin(ALPHA_BASE), d['L_0_A1']],    # [Angle X, Angle Y, Angle Z] position A
            [xy_B*np.cos(ALPHA_BASE), xy_B*np.sin(ALPHA_BASE), z_B],    # [Angle X, Angle Y, Angle Z] position B
            [xy_C*np.cos(ALPHA_BASE), xy_C*np.sin(ALPHA_BASE), z_C],    # [Angle X, Angle Y, Angle Z] position C
            [X, Y, Z]])                                                 # [Angle X, Angle Y, Angle Z] final position
    return (X,Y,Z,end_alpha,XYZ)                                       # Create matrix with positions of all points


def an_invkin_iterative(end_X, end_Y, end_Z):
    # Base angle calculation
    Alpha_Base = math.atan2(end_Y, end_X)

    # Set search parameters
    steps = 50  # Resolution of the search space
    best_solution = None
    min_error = float('inf')

    # Loop through all possible combinations of Alpha_A, Alpha_B, and Alpha_C
    for Alpha_A in np.linspace(Alpha_A_min, Alpha_A_max, steps):
        for Alpha_B in np.linspace(Alpha_B_min, Alpha_B_max, steps):
            for Alpha_C in np.linspace(Alpha_C_min, Alpha_C_max, steps):

                # Calculate position with these angles
                X_calc, Y_calc, Z_calc, _, _ = an_kin(Alpha_A, Alpha_B, Alpha_C, Alpha_Base)

                # Calculate the error (distance to the desired position)
                error = np.sqrt((X_calc - end_X)**2 + (Y_calc - end_Y)**2 + (Z_calc - end_Z)**2)

                # Update the best solution if a better configuration is found
                if error < min_error:
                    min_error = error
                    best_solution = (Alpha_A, Alpha_B, Alpha_C)
        
    # Check if a solution is found
    if best_solution and min_error <= d['max_error']:
        Alpha_A, Alpha_B, Alpha_C = best_solution
    else:
        Alpha_A, Alpha_B, Alpha_C = None, None, None

    return Alpha_A, Alpha_B, Alpha_C, Alpha_Base

# CALL FUNCTIONS TO CALCULATE MOVEMENT OF ROBOT ARM
print(f"Input coordinates: X = {end_X} mm  Y = {end_Y} mm  Z = {end_Z} mm")
print ("")
[Alpha_A, Alpha_B, Alpha_C, Alpha_Base] = an_invkin_iterative(end_X, end_Y, end_Z)

if Alpha_A is not None:
    print(f"resulting segment angles - Alpha_Base = {np.rad2deg(Alpha_Base):.2f}\N{DEGREE SIGN}")
    print(f"resulting segment angles - Alpha_A    = {np.rad2deg(Alpha_A):.2f}\N{DEGREE SIGN}")
    print(f"resulting segment angles - Alpha_B    = {np.rad2deg(Alpha_B):.2f}\N{DEGREE SIGN}")
    print(f"resulting segment angles - Alpha_C    = {np.rad2deg(Alpha_C):.2f}\N{DEGREE SIGN}")
    
    X, Y, Z, end_alpha, XYZ = an_kin(Alpha_A, Alpha_B, Alpha_C, Alpha_Base)
    print ("")
    print(f"End coordinates: X = {X:.3f} mm  Y = {Y:.3f} mm  Z = {Z:.3f} mm")

else:
    print("Endpoint cannot be reached.")

# Create the figure and subplots with a custom grid
fig, axs = plt.subplots(2, 2, figsize=(10, 8))  # 2x2 grid
axs = axs.flatten()

# XY plane (top view) - top left
axs[0].plot(XYZ[:, 0], XYZ[:, 1], 'o-', label='XY projection')
axs[0].set_title('XY plane (top view)')
axs[0].set_xlabel('X [mm]')
axs[0].set_ylabel('Y [mm]')
axs[0].grid()
axs[0].legend()

# XZ plane (front view) - bottom left
axs[2].plot(XYZ[:, 0], XYZ[:, 2], 'o-', label='XZ projection')
axs[2].set_title('XZ plane (front view)')
axs[2].set_xlabel('X [mm]')
axs[2].set_ylabel('Z [mm]')
axs[2].grid()
axs[2].legend()

# YZ plane (side view) - bottom right
axs[3].plot(XYZ[:, 1], XYZ[:, 2], 'o-', label='YZ projection')
axs[3].set_title('YZ plane (side view)')
axs[3].set_xlabel('Y [mm]')
axs[3].set_ylabel('Z [mm]')
axs[3].grid()
axs[3].legend()

# Remove unused subplot (top right)
fig.delaxes(axs[1])

# Set axis limits (for all axes)
# Adjust limits as needed
axs[0].set_xlim(-50, 300) # set x-axis top view
axs[0].set_ylim(-50, 300) # set y-axis top view

axs[2].set_xlim(-50, 300) # set x-axis front view
axs[2].set_ylim(300, -50) # set z-axis front view

axs[3].set_xlim(-50, 300) # set z-axis side view
axs[3].set_ylim(300, -50) # set y-axis side view

# axs[x].set_ylim(100, -10)

# Adjust layout for a cleaner display
plt.tight_layout()
plt.show()



''' DETERMINE OIL VOLUME IN CYLINDERS '''

# Import data from angle sensors - for now manual input
Alpha_A_sens = np.deg2rad(70)       #degrees
Alpha_B_sens = np.deg2rad(-54)      #degrees
Alpha_C_sens = np.deg2rad(30)       #degrees


# # CYLINDER A - REQUIRED VOLUME FOR CORRECT ANGLE
Pos_C_A_target = np.sqrt(L_A_a2**2 + L_A_a3**2 - (2*L_A_a2 * L_A_a3 * np.cos(alpha_A1_rad + Alpha_A - alpha_A3_rad)))
V_oil_A_piston_target = (d['C_A_dia_piston']/2)**2*np.pi*d['C_A_stroke']*((Pos_C_A_target-d['C_A_min'])/(C_A_max-d['C_A_min']))
V_oil_A_rod_target = (((d['C_A_dia_piston']/2)**2*np.pi)-((d['C_A_dia_rod']/2)**2*np.pi)) * d['C_A_stroke'] *(1-((Pos_C_A_target-d['C_A_min'])/(C_A_max-d['C_A_min'])))
    

Pos_C_A_Current = np.sqrt(L_A_a2**2 + L_A_a3**2 - (2*L_A_a2 * L_A_a3 * np.cos(alpha_A1_rad + Alpha_A_sens - alpha_A3_rad)))
V_oil_A_piston_Current = (d['C_A_dia_piston']/2)**2*np.pi*d['C_A_stroke']*((Pos_C_A_Current-d['C_A_min'])/(C_A_max-d['C_A_min']))
V_oil_A_rod_Current = (((d['C_A_dia_piston']/2)**2*np.pi)-((d['C_A_dia_rod']/2)**2*np.pi)) * d['C_A_stroke'] *(1-((Pos_C_A_Current-d['C_A_min'])/(C_A_max-d['C_A_min'])))


Pos_C_A_delta = Pos_C_A_target - Pos_C_A_Current 
V_oil_A_piston_delta = (V_oil_A_piston_target - V_oil_A_piston_Current)
V_oil_A_rod_delta = (V_oil_A_rod_target - V_oil_A_rod_Current)

print ("")
print (f"Volume change Cylinder A - piston side = {V_oil_A_piston_delta:.2f} mm^3")
print (f"Volume change Cylinder A - rod side    = {V_oil_A_rod_delta:.2f} mm^3")

# # CYLINDER B - REQUIRED VOLUME FOR CORRECT ANGLE
Pos_C_B_target = np.sqrt(L_B_b2**2 + L_B_b3**2 - (2*L_B_b2 * L_B_b3 * np.cos(np.pi - alpha_B1_rad + Alpha_B - alpha_B3_rad)))
V_oil_B_piston_target = (d['C_B_dia_piston']/2)**2*np.pi*d['C_B_stroke']*((Pos_C_B_target-d['C_B_min'])/(C_B_max-d['C_B_min']))
V_oil_B_rod_target = ((d['C_B_dia_piston']/2)**2*np.pi-(d['C_B_dia_rod']/2)**2*np.pi) * d['C_B_stroke'] *(1-((Pos_C_B_target-d['C_B_min'])/(C_B_max-d['C_B_min'])))
    

Pos_C_B_Current = np.sqrt(L_B_b2**2 + L_B_b3**2 - (2*L_B_b2 * L_B_b3 * np.cos(np.pi - alpha_B1_rad + Alpha_B_sens - alpha_B3_rad)))
V_oil_B_piston_Current = (d['C_B_dia_piston']/2)**2*np.pi*d['C_B_stroke']*((Pos_C_B_Current-d['C_B_min'])/(C_B_max-d['C_B_min']))
V_oil_B_rod_Current = (((d['C_B_dia_piston']/2)**2*np.pi)-(d['C_B_dia_rod']/2)**2*np.pi) * d['C_B_stroke'] *(1-((Pos_C_B_Current-d['C_B_min'])/(C_B_max-d['C_B_min'])))
 

Pos_C_B_delta = Pos_C_B_target - Pos_C_B_Current 
V_oil_B_piston_delta = (V_oil_B_piston_target - V_oil_B_piston_Current)
V_oil_B_rod_delta = (V_oil_B_rod_target - V_oil_B_rod_Current)

print (f"Volume change Cylinder B - piston side = {V_oil_B_piston_delta:.2f} mm^3")
print (f"Volume change Cylinder B - rod side    = {V_oil_B_rod_delta:.2f} mm^3")

# # CYLINDER C - REQUIRED VOLUME FOR CORRECT ANGLE
Pos_C_C_target = np.sqrt(L_C_c2**2 + L_C_c3**2 - (2*L_C_c2 * L_C_c3 * np.cos(np.pi - alpha_C1_rad - Alpha_C - alpha_C3_rad)))
V_oil_C_piston_target = (d['C_C_dia_piston']/2)**2*np.pi*d['C_C_stroke']*((Pos_C_C_target-d['C_C_min'])/(C_C_max-d['C_C_min']))
V_oil_C_rod_target = (((d['C_C_dia_piston']/2)**2*np.pi)-(d['C_C_dia_rod']/2)**2*np.pi) * d['C_C_stroke'] *(1-((Pos_C_C_target-d['C_C_min'])/(C_C_max-d['C_C_min'])))
    

Pos_C_C_Current = np.sqrt(L_C_c2**2 + L_C_c3**2 - (2*L_C_c2 * L_C_c3 * np.cos(np.pi - alpha_C1_rad - Alpha_C_sens - alpha_C3_rad)))
V_oil_C_piston_Current = (d['C_C_dia_piston']/2)**2*np.pi*d['C_C_stroke']*((Pos_C_C_Current-d['C_C_min'])/(C_C_max-d['C_C_min']))
V_oil_C_rod_Current = (((d['C_C_dia_piston']/2)**2*np.pi)-(d['C_C_dia_rod']/2)**2*np.pi) * d['C_C_stroke'] *(1-((Pos_C_C_Current-d['C_C_min'])/(C_C_max-d['C_C_min'])))
 

Pos_C_C_delta = Pos_C_C_target - Pos_C_C_Current
V_oil_C_piston_delta = (V_oil_C_piston_target - V_oil_C_piston_Current)
V_oil_C_rod_delta = (V_oil_C_rod_target - V_oil_C_rod_Current)

print (f"Volume change Cylinder B - piston side = {V_oil_C_piston_delta:.2f} mm^3")
print (f"Volume change Cylinder B - rod side    = {V_oil_C_rod_delta:.2f} mm^3")



'''
PLOTS ANIMATION ROBOT ARM 3D
'''


# 3D PLOT ROBOT ARM
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the coordinates of the robot arm
ax.plot(XYZ[:,0], XYZ[:,1], XYZ[:,2], c='b', marker='o', label='ARM')

# Labels en title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title(f"3D plot of robot arm at position (X: {end_X}, Y: {end_Y}, Z: {end_Z})")

# Axis limits
ax.set_xlim([-150, 250])  # X-axis from -150 to 250
ax.set_ylim([-150, 250])  # Y-axis from -150 to 250
ax.set_zlim([0, 400])     # Z-axis from 0 to 400

# Legend and grid settings
ax.legend(loc='center left', bbox_to_anchor=(1.1, 0.1))
ax.invert_zaxis()  # Reverse Z-axis
plt.grid()
plt.show()


# ANIMATION OF ROBOT ARM IN 3D
from matplotlib.animation import FuncAnimation

# Calculate the average angles for the starting position
Alpha_Base_avg = 0  # No rotation in the base for the starting position
_, _, _, _, start_XYZ = an_kin(Alpha_A_avg, Alpha_B_avg, Alpha_C_avg, Alpha_Base_avg)


# Animation settings
steps = 100  # Number of frames per movement
XYZ_frames = []

# Interpolatie voor bewegingen
# Interpolation for movements
for t in np.linspace(0, 1, steps):
    interp_Base = t * Alpha_Base
    X, Y, Z, _, frame_XYZ = an_kin(Alpha_A_avg, Alpha_B_avg, Alpha_C_avg, interp_Base)
    XYZ_frames.append(frame_XYZ)

for t in np.linspace(0, 1, steps):
    interp_A = Alpha_A_avg + t * (Alpha_A - Alpha_A_avg)
    X, Y, Z, _, frame_XYZ = an_kin(interp_A, Alpha_B_avg, Alpha_C_avg, Alpha_Base)
    XYZ_frames.append(frame_XYZ)

for t in np.linspace(0, 1, steps):
    interp_B = Alpha_B_avg + t * (Alpha_B - Alpha_B_avg)
    X, Y, Z, _, frame_XYZ = an_kin(Alpha_A, interp_B, Alpha_C_avg, Alpha_Base)
    XYZ_frames.append(frame_XYZ)

for t in np.linspace(0, 1, steps):
    interp_C = Alpha_C_avg + t * (Alpha_C - Alpha_C_avg)
    X, Y, Z, _, frame_XYZ = an_kin(Alpha_A, Alpha_B, interp_C, Alpha_Base)
    XYZ_frames.append(frame_XYZ)

# Set the target point
target_point = (end_X, end_Y, end_Z)

# Setup for the animation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Axis limits
ax.set_xlim([-150, 250])
ax.set_ylim([-150, 250])
ax.set_zlim([0, 400])
ax.invert_zaxis()  # Reverse Z-axis

# Labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title(f"3D animation of robot arm moving to (X: {end_X}, Y: {end_Y}, Z: {end_Z})")

# Initialize line for animation (path)
line, = ax.plot([], [], [], 'bo-', label='Movement')
path_x, path_y, path_z = [], [], []  # To track the path

# Plot the target point
ax.scatter(target_point[0], target_point[1], target_point[2], color='red', s=100, label="Target Point")

# Temporary list to store arm segments
arm_lines = []

def draw_arm(frame):
    # Get the XYZ coordinates of the joints
    X, Y, Z = XYZ_frames[frame][:, 0], XYZ_frames[frame][:, 1], XYZ_frames[frame][:, 2]
    
    # Remove old arm segments
    for line in arm_lines:
        line.remove()
    arm_lines.clear()

    # Draw the arm segments (lines between joints)
    for i in range(1, len(X)):
        line, = ax.plot(
            [X[i-1], X[i]],
            [Y[i-1], Y[i]],
            [Z[i-1], Z[i]],
            color='b',
            marker='o',
            markersize=5,
        )
        arm_lines.append(line)

def update(frame):
    # Get the XYZ coordinates of the frame
    x, y, z = XYZ_frames[frame][:, 0], XYZ_frames[frame][:, 1], XYZ_frames[frame][:, 2]
    
    # Add the current position to the path
    path_x.append(x[-1])  # End x-coordinate
    path_y.append(y[-1])  # End y-coordinate
    path_z.append(z[-1])  # End z-coordinate

    # Update the path line
    line.set_data(path_x, path_y)
    line.set_3d_properties(path_z)
    
    # Draw the arm
    draw_arm(frame)

    return line,

# Start the animation without blit
ani = FuncAnimation(fig, update, frames=len(XYZ_frames), interval=50, blit=False)

# Show the animation
plt.legend()
plt.show()
plt.pause(0.1)  # Ensure rendering completes correctly
