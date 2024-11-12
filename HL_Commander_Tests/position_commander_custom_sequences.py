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
import numpy as np
import time

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')


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
    z = np.linspace(0.5, 1, num_points)  # Vary height from 0.5 to 1 meter
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

#####################################
#                                   #
#          Square Sleep Sequence    #
#                                   #
#####################################

def square_sequence_with_pauses(pc):

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf,
                x=0.0, y=0.0, z=0.0,
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

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # simple_sequence()
    # slightly_more_complex_usage()
    # land_on_elevated_surface()
    figure_8_sequence()
    square_sequence_with_pauses()