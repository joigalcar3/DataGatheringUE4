#!/usr/bin/env python
"""
Provides the procedural code in order to scope any signals given a specific command to the drone.

It is used to easily visualize the functionality of the scoping function within the ControllerTuning class and for
debugging purposes.
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Stable"

# Imports
import airsim
import keyboard
from math import *
from utils import depickle
from Occupancy_grid.user_input import load_user_input
from Drone_flight.ControllerTuning import ControllerTuning


if __name__ == "__main__":
    user_input = load_user_input()
    user_input.save_scope_images = True

    # Obtain AirSim client object
    client = airsim.MultirotorClient()
    controller_tuning_switch = True

    # Activate the controller tuning object which allows signal scoping
    controller = ControllerTuning(user_input, client, controller_tuning_switch=controller_tuning_switch)

    # HERE YOU CAN WRITE A SET OF ACTIONS THAT THE DRONE SHOULD FOLLOW. An example is shown next
    pose = client.simGetVehiclePose()
    client.enableApiControl(True)
    pose.orientation.z_val = sin(pi/8)
    pose.orientation.w_val = cos(pi/8)
    client.setTeleportYawRef(degrees(pi/8))
    client.simSetVehiclePose(pose, True)
    client.moveOnPathAsync([airsim.Vector3r(1,1,0)], 1, 10000, airsim.DrivetrainType.ForwardOnly,
                           airsim.YawMode(False, 0), 2, 1)

    # Start collecting data from the simulation
    controller.initialize_data_gathering()
    while True:
        # Manual break in the collection of data
        if keyboard.is_pressed('K'):
            print('The letter K has been pressed.')
            break

    # Run the sequence of controller functions to store the data
    controller.collect_data_gathered()
    controller.scope_plotting_signals()
    controller.clean_data_gathered()

    # Code used to load an already created figure
    store_location = user_input.scope_images_remote_store_location
    filename = 'pos_error_x-pos_error_y-pos_error_z.fig.pickle'
    file_contents = depickle(store_location, filename)
