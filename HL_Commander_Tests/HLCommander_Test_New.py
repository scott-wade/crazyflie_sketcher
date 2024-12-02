# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
This script shows the basic use of the PositionHlCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.

The PositionHlCommander uses position setpoints.

Change the URI variable to your Crazyflie configuration.
"""

#https://www.youtube.com/watch?v=6s8i-nhPjt0&list=PLj9XMmQVSr-C-ZXeWWyZgeZ7C4g6A5nPj&index=3&t=2422s&ab_channel=Bitcraze
#58:00
#Sends one command, not every x ms (less packets)
#Planning happens onboard Crazyflie
#Works well with MoCap system NOT with optical flow deck

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

#####################################
#                                   #
#          Added libraries          #
#                                   #
#####################################
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
import numpy as np
import time
import logging
import csv
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

file_name = "output.csv"
# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')



def slightly_more_complex_usage():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf,
                x=0.0, y=0.0, z=0.0,
                default_velocity=0.3,
                default_height=0.5,
                controller=PositionHlCommander.CONTROLLER_PID) as pc:
            # Go to a coordinate
            pc.go_to(1.0, 1.0, 1.0)

            # Move relative to the current position
            pc.right(1.0)

            # Go to a coordinate and use default height
            pc.go_to(0.0, 0.0)

            # Go slowly to a coordinate
            pc.go_to(1.0, 1.0, velocity=0.2)

            # Set new default velocity and height
            pc.set_default_velocity(0.3)
            pc.set_default_height(1.0)
            pc.go_to(0.0, 0.0)

def land_on_elevated_surface():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(scf,
                                 default_height=0.5,
                                 default_velocity=0.2,
                                 default_landing_height=0.35,
                                 controller=PositionHlCommander.CONTROLLER_PID) as pc:
            # fly onto a landing platform at non-zero height (ex: from floor to desk, etc)
            pc.forward(1.0)
            pc.left(1.0)
            # land() will be called on context exit, gradually lowering to default_landing_height, then stopping motors


def simple_sequence():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_PID) as pc:
            pc.forward(1.0)
            pc.left(1.0)
            pc.back(1.0)
            pc.go_to(0.0, 0.0, 1.0)

#####################################
#                                   #
#          Figure 8 Sequence        #
#                                   #
#####################################

def generate_figure_8_points(a=1, num_points=100):
    t = np.linspace(0, 2 * np.pi, num_points)
    x = a * np.sin(t)
    y = a * np.sin(t) * np.cos(t)
    z = np.linspace(0.5, 0.5, num_points)  # Vary height from 0.5 to 1 meter
    return x, y, z

def figure_8_sequence():

    # Generate figure-8 coordinates
    x, y, z = generate_figure_8_points(a=0.5, num_points=50)  # Adjust 'a' for size and num_points as needed
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf,
                x=0.0, y=0.0, z=0.0,
                default_velocity=0.2,
                default_height=0.5, #TODO: match height
                controller=PositionHlCommander.CONTROLLER_PID) as pc:
            # Go to starting position
            pc.go_to(x[0], y[0], z[0])

            # Follow the figure-8 path
            for xi, yi, zi in zip(x[1:], y[1:], z[1:]):
                pc.go_to(xi, yi, zi)

            # Return to starting position
            pc.go_to(0.0, 0.0, 0.5)

def init_lg_stab():
    lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')
    return lg_stab
def calculate_tilt_angle(x, y, z):
    return np.degrees(np.arctan2(np.sqrt(x**2 + y**2), z))

def figure_8_sequence_logging():
    # Generate figure-8 coordinates
    # Initialize empty lists for x and y coordinates
    x = []
    y = []
    z = []
    x_dot = []
    y_dot = []
    z_dot =[]
    scale = 2
    offset = 0.1
    # Read the CSV file and populate the lists
    with open(file_name, 'r') as file:
        next(file)  # Skip the header row
        for line in file:
            values = line.strip().split(',')
           
            x.append(float(values[0])/scale +offset)
            y.append(float(values[1])/scale +offset)
            z.append(float(values[2]))
            x_dot.append(float(values[7]))
            y_dot.append(float(values[8]))
            z_dot.append(float(values[9]))

    # Print the first few elements to verify
    print("First 5 x-coordinates:", x[:5])
    print("First 5 y-coordinates:", y[:5])
    # Initialize an empty list for velocities
    velocities = []

    # # Read the CSV file and populate the velocities list
    # with open('MJ_traj_vel.csv', 'r') as file:
    #     next(file)  # Skip the header row
    #     for line in file:
    #         values = line.strip().split(',')
    #         if len(values) == 3:
    #             x_vel = float(values[1])
    #             y_vel = float(values[2])
                
    #             velocities.append(norm_vel)  # Append the velocity value
    x_dot = np.array([x_dot])
    y_dot = np.array([y_dot])
    z_dot = np.array([z_dot])
    norm_vel = (x_dot**2 + y_dot**2 +z_dot**2)**(0.5)
    norm_vel = norm_vel.tolist()
    lg_stab = init_lg_stab()

    # Initialize vectors to store state estimates and errors
    state_estimates = []
    errors = []
    z_offset = 0.5
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        with PositionHlCommander(
                scf,
                default_velocity=0.2,
                default_height=z_offset, #TODO: match height
                controller=PositionHlCommander.CONTROLLER_PID) as pc:
            # Go to starting position
            duration = 10
            end_time = time.time() + duration
            # while(time.time()<end_time):
            #     pc.go_to(pc.go_to(0, 0, z_offset))
            #     if tilt_angle > 20:
            #         # Turn off motors
            #         scf.cf.commander.send_stop_setpoint()
            #         print("Propellers turned off: Tilt angle exceeded 20 degrees")
            #         break   
            #     time.sleep(0.01)
            # end_time = time.time() + duration
            # pc.go_to(x[0], y[0], z_offset)
            # while(time.time()<end_time):
            #     pc.go_to(x[0], y[0], z_offset)
            #     attitude = scf.cf.state
            #     tilt_angle = calculate_tilt_angle(attitude['roll'], attitude['pitch'], 1)
            #     if tilt_angle > 20:
            #         # Turn off motors
            #         scf.cf.commander.send_stop_setpoint()
            #         print("Propellers turned off: Tilt angle exceeded 20 degrees")
            #         break
            #     time.sleep(0.01)  
            print(x)
            pc.go_to(x[0], y[0], z_offset)
            print("git got")
            for xi, yi, zi, vi in zip(x, y, z, norm_vel[0]):
                time.sleep(0.01)
                if(vi==0):
                    vi = .2
                 # Calculate tilt angle
                # attitude = scf.cf.state
                # tilt_angle = calculate_tilt_angle(attitude['roll'], attitude['pitch'], 1)

                # # Check if tilt angle exceeds 20 degrees
                # if tilt_angle > 20:
                #     # Turn off motors
                #     scf.cf.commander.send_stop_setpoint()
                #     print("Propellers turned off: Tilt angle exceeded 20 degrees")
                #     break     
                print(vi)   
                pc.go_to(xi, yi,z=zi)#, velocity=abs(vi))        
                
                # with SyncLogger(scf, lg_stab) as logger:
                #     for log_entry in logger:
                #         timestamp = log_entry[0]
                #         data = log_entry[1]
                #         logconf_name = log_entry[2]

                #         # Store state estimate
                #         state_estimate = [data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']]
                #         state_estimates.append(state_estimate)

                #         # Calculate and store error
                #         error = [xi - state_estimate[0], yi - state_estimate[1], vi - state_estimate[2]]
                #         errors.append(error)

                #         print(f'[{timestamp}][{logconf_name}]: State: {state_estimate}, Error: {error}')

                        
                #         # Only log one entry per setpoint
                #         break
            
            # Return to starting position
            pc.go_to(0.0, 0.0, 0.5)

    # Convert lists to numpy arrays for easier manipulation
    state_estimates = np.array(state_estimates)
    errors = np.array(errors)

    #expect (num_points, 3)
    # print("State Estimates shape:", state_estimates.shape)
    # print("Errors shape:", errors.shape)

    # Write data to CSV file
    with open('figure8_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x_estimate', 'y_estimate', 'z_estimate', 'x_error', 'y_error', 'z_error'])
        for state, error in zip(state_estimates, errors):
            writer.writerow(list(state) + list(error))

    print("Data has been written to figure8_data.csv")

#####################################
#                                   #
#          Square Sleep Sequence    #
#                                   #
#####################################

def square_sequence_with_pauses(pc):

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf,
                default_velocity=0.2,
                default_height=0.5, #TODO: match height
                controller=PositionHlCommander.CONTROLLER_PID) as pc:
            
            # Define the corners of the square
            corners = [
                (0.0, 0.0, 0.5),  # Bottom left
                (1.0, 0.0, 0.5),  # Bottom right
                (1.0, 1.0, 0.5),  # Top right
                (0.0, 1.0, 0.5)   # Top left
            ]

            # Sequence of movements with pauses at each corner
            for x, y, z in corners:
                pc.go_to(x, y, z)  # Go to the corner
                time.sleep(2)  # Pause for 2 seconds at each corner

            # Return to the starting position
            pc.go_to(0.0, 0.0, 0.5)


#####################################
#                                   #
#         Controller                #
#                                   #
#####################################

#After attempting to go_to XYZ_ref, log values
#log XYZ_actual, and calculate and store error
#Pass XYZ_actual_i and XYZ_ref_i:i+N (or just timestep) to MPC/minimum jerk 
#Find new X_bar and go_to
#Repeat

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # simple_sequence()
    # slightly_more_complex_usage()
    # land_on_elevated_surface()
    #figure_8_sequence()
    figure_8_sequence_logging()
    square_sequence_with_pauses()
