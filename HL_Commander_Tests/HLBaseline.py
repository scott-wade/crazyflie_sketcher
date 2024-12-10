# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2023 Bitcraze AB
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

# https://github.com/USC-ACTLab/crazyswarm/discussions/466 - goto max timing
# https://forum.bitcraze.io/viewtopic.php?t=858 - fixed commander frequency
# https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/python_api/ - logging
# https://forum.bitcraze.io/viewtopic.php?t=4125 - fixed commander frequency


"""
Shows how to send full state control setpoints to the Crazyflie
"""
import logging
import time
import csv
import argparse



from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander
import numpy as np


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')
logging.basicConfig(level=logging.ERROR)

# ==============================
# Global Position and Control Values
# ==============================

#These values are measured before run
# #Original values
# x_offset = 0.25
# y_offset = 0.5
# z_offset = -0.13
x_offset = 0.3
y_offset = 0.0
z_offset = -0.18
file_name = ''

#These values are updated within position_callback
set_initial_est = False
x_est = 0.0
y_est = 0.0
z_est = 0.0
curr_x = 0.0
curr_y = 0.0
curr_z = 0.0
dt = 0.01 # 100Hz

#This value prevents the HL Commander goto and the first LL command from overshooting
initial_z_offset = z_offset+0.018

# ==============================
# Math Helper Functions
# ==============================

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to quaternion

    Args:
        roll (float): roll, in radians
        pitch (float): pitch, in radians
        yaw (float): yaw, in radians

    Returns:
        array: the quaternion [x, y, z, w]
    """
    return Rotation.from_euler(seq='xyz', angles=(roll, pitch, yaw), degrees=False).as_quat()

# ==============================
# Estimator and Logging Functions
# ==============================

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=10)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

 # Define a callback function to handle the logged data

#This is called whenever new position data is received
def position_callback(timestamp, data, logconf):
    global x_est, y_est, z_est
    global curr_x, curr_y, curr_z
    global set_initial_est
    
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    curr_x = x
    curr_y = y
    curr_z = z
    
    if set_initial_est == False:
        x_est = x
        y_est = y
        z_est = z
        set_initial_est = True
        
    # print('pos: ({}, {}, {})'.format(x, y, z))

# def get_current_state_estimates(scf):
#     # Set up logging configuration
#     log_conf = LogConfig(name='Position', period_in_ms=500)
#     log_conf.add_variable('kalman.stateX', 'float')
#     log_conf.add_variable('kalman.stateY', 'float')
#     log_conf.add_variable('kalman.stateZ', 'float')

#     # Add the log config to the Crazyflie
#     scf.cf.log.add_config(log_conf)
#     log_conf.data_received_cb.add_callback(position_callback)
#     # Start logging
#     log_conf.start()
#     # Wait for a short time to get the position
#     time.sleep(1)

#     # Stop logging
#     log_conf.stop()

# ==============================
# Setpoint Functions
# ==============================

def send_continuous_setpoint(cf, duration, pos, vel, acc, orientation, rollrate, pitchrate, yawrate):
    # Set points must be sent continuously to the Crazyflie, if not it will think that connection is lost
    end_time = time.time() + duration
    while time.time() < end_time:
        cf.commander.send_full_state_setpoint(pos, vel, acc, orientation, rollrate, pitchrate, yawrate)
        # time.sleep(0.2)

# ==============================
# Main Control Sequence
# ==============================

def parse_MPC_output(filename):
    # Open the CSV file and read its contents
    Xref = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader, None)  # skip the headers
        # Iterate through each row in the CSV file
        for row in csv_reader:
            row2 = row.copy()
            for x,element in enumerate(row):
                if element:
                    row2[x] = float(element)
                else:
                    row2[x] = 0.0  # or any default value
            # Append each row to the array
            Xref.append((row2))
    print(len(Xref))
    

    # print(accelarr.shape)
    # print(np.array(Xref).shape)
    # print(output.shape)
    return Xref

def traj2ref(Xref):
    # takes Xref in format [pos, quat, vel, rollrate, accel]
    # updates the pos_x, pos_y, and pos_z values with the position offsets and initial estimates
    # returns in format [[pos], [vel], [acc], [ori], [rollrate], [pitchrate], [yawrate]]
    
    global x_offset, y_offset, z_offset, x_est, y_est, z_est
    
    
   
    pos = [
        x_offset + Xref[0],
        y_offset+Xref[1],
        z_offset
    ]

   
    
    return pos

def run_MPC_sequence(scf, log_conf):

    #Parse MPC output and store states 
    csv_output = parse_MPC_output(file_name)

    #HL fly to board
    print("Flying to board via HLCommander...\n")
    pos = traj2ref(csv_output[0])
    pc = PositionHlCommander(
                scf,
                x=0.0, y=0.0, z=0.0,
                default_velocity=0.5,
                default_height=0.25, 
                controller=PositionHlCommander.CONTROLLER_PID)
        # Go to starting position
    print("Taking off...\n")
    pc.take_off(velocity=0.5)
    print("Flying to board via HLCommander...\n")
   
    #Change z_offset for first point
    pos[2] = initial_z_offset
    print(f"Going to position x: {pos[0]} y: {pos[1]} z: {pos[2]}\n")

    pc.go_to(pos[0], pos[1], pos[2], velocity=0.5)

    print("Reached starting point!\n")

   
    
    # start following the trajectory
    state_estimates = []
    errors = []
    for full_state in csv_output:
      
        pos = traj2ref(full_state)

        # Follow the figure-8 path
        pc.go_to(pos[0], pos[1], pos[2])
        # print('Set point: ({}, {}, {})'.format(pos[0], pos[1], pos[2]))
        state_estimates.append([curr_x,curr_y,curr_z])
        errors.append([curr_x-pos[0],curr_y-pos[1],curr_z-pos[2]])
       
    
    with open('results_HLCommander.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x_estimate', 'y_estimate', 'z_estimate', 'x_error', 'y_error', 'z_error'])
        for state, error in zip(state_estimates, errors):
            writer.writerow(list(state) + list(error))

    
    
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    #cf.commander.send_notify_setpoint_stop()
    pc.go_to(.2,.2,.2, velocity=0.1)
    pc.go_to(0.05,0.05,.05, velocity=0.1)
    pc.go_to(0.05,0.05,.05, velocity=0.1)
    pc.land()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    parser = argparse.ArgumentParser("simple_example")
    parser.add_argument("input_file", help="An integer will be increased by 1 and printed.", type=str)
    args = parser.parse_args()
    file_name = args.input_file
    print(file_name)
    print(f"Using file: {file_name}\n")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)  
        
        log_conf = LogConfig(name='Position', period_in_ms=10)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        print('-----POSITION PID-----')
        # OG: 2.0
        pzKp = scf.cf.param.get_value('posCtlPid.zKp')
        print("Current pzKi:", pzKp)

        new_pzKp = float(pzKp)#+1
        scf.cf.param.set_value('posCtlPid.zKp', new_pzKp)
        print("Updated pzKp:", new_pzKp)

        # OG: 0.5
        pzKi = scf.cf.param.get_value('posCtlPid.zKi')
        print("Current pzKi:", pzKi)

        new_pzKi = float(pzKi) #+0.15 # write new value here
        scf.cf.param.set_value('posCtlPid.zKi', new_pzKp)
        print("Updated pzKi:", new_pzKi)

        print('-----VELOCITY PID-----')

        # OG: 25.0
        vzKp = scf.cf.param.get_value('velCtlPid.vzKp')
        print("Current vzKp:", vzKp)

        new_vzKp = vzKp # write new value here
        scf.cf.param.get_value('velCtlPid.vzKp', new_vzKp)
        print("Updated vzKi:", new_vzKp)

        # OG: 15.0
        vzKi = scf.cf.param.get_value('velCtlPid.vzKi')
        print("Current vzKi:", vzKi)

        new_vzKi = vzKi
        scf.cf.param.set_value('velCtlPid.vzKi', new_vzKi)
        print("Updated vzKi:", new_vzKi)
        
        # Add the log config to the Crazyflie
        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(position_callback)
        # Start logging
        log_conf.start()
        # Wait for a short time to get the position
        time.sleep(1)
    
        #get_current_state_estimates(scf)
        run_MPC_sequence(scf, log_conf)
        
        # Stop logging
        log_conf.stop()


    # Write data to CSV file
        print("Data has been written to figure8_data.csv")
       