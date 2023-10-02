#!/usr/bin/env python
"""
Provides the main file that the user should run in order to interact with Unreal Engine 4 and collect flight data.

The user should already be running Unreal Engine 4 in the background.
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
from _init_paths import init_paths  # Setup the system paths
init_paths()

# Import self created libraries
from user_input import load_user_input
from _init_json_config import find_config_json
from Drone_flight.Data_gathering.DataGathering import DataGathering

# Import Python libraries
import sys
import random
import numpy as np
from icecream import ic
import matplotlib as mpl

np.random.seed(10)
random.seed(10)

# Set up the plotting backend
mpl.use('TKAgg')


if __name__ == '__main__':
    # Load the user input
    args = load_user_input()

    # Obtain the location of the Airsim json configuration file and retrieve the data
    location_json_file, data = find_config_json(args)

    # Debugging with icecream
    gettrace = getattr(sys, 'gettrace', None)
    if gettrace() and not args.PSO_tuning_switch:
        ic.enable()
    else:
        ic.disable()

    # Retrieving the start position of the drone
    vehicle_name = list(data['Vehicles'].keys())[0]
    coord = data['Vehicles'][vehicle_name]
    start_coord = (coord['X'], coord['Y'], coord['Z'])

    data_gathering = DataGathering(data, args, vehicle_name=vehicle_name, vehicle_start_position=start_coord)
    if args.PSO_tuning_switch:
        data_gathering.pso_pid_tuning()
    else:
        data_gathering.gather_data_consecutive_runs()
