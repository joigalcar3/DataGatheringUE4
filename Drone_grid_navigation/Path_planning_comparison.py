#!/usr/bin/env python
"""
Provides the code for the generation of histograms that allow the comparison of the computational performance of the
different navigation approaches considered. With computational performance it is referred to computational time.

A* is used as baseline as it is expected to be the most efficient for the chosen metric. Finally, the user can find the
plot generated in Figure 8.29 of the author's master thesis.
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
import sys
import time
import random
import pickle
import statistics
import matplotlib
import numpy as np
from icecream import ic
import matplotlib as mpl
import matplotlib.pyplot as plt

from user_input import load_user_input
from _init_json_config import find_config_json
from Drone_flight.DroneFlight import DroneFlight
from Environment_extraction.OccupancyMap import OccupancyMap
from Drone_grid_navigation.GridNavigation import GridNavigation

# Plotting settings
mpl.rcParams['font.family'] = 'Arial'
mpl.rcParams['grid.alpha'] = 0.5
mpl.use('TkAgg')
font = {'size': 30,
        'family': "Arial"}
mpl.rc('font', **font)

# Whether the simulations need to be run or the computation time information has already been obtained and is saved in a
# directory waiting to be loaded
recompute_statistics = False
if recompute_statistics:
    # Obtain the user input and the information stored in the settings json file
    args = load_user_input()
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

    # Obtaining the clock speed information
    clock_speed = data['ClockSpeed']
    sample_rates = {key: value * clock_speed for key, value in {}.items()}

    # Create a Drone
    # Creating a drone flight to exploit some of the functions
    drone_flight = DroneFlight(args, sample_rates, clock_speed=clock_speed, vehicle_name=vehicle_name,
                               vehicle_start_position=start_coord)

    # Parameters that define the performance comparison
    n_iterations = 100
    PP_computation_time = {"A_star": [], "Wavefront": [], "Voronoi": [], "RRT*": [], "PRM": []}
    for run in range(n_iterations):
        # Obtaining the occupancy grid
        drone_flight.altitude_m = -random.randint(3, 11)
        drone_flight.altitude = drone_flight.distance_to_ue4(drone_flight.altitude_m, True)

        # Extract occupancy grid
        drone_flight.env_map = OccupancyMap(cell_size=drone_flight.cell_size,
                                            ue4_airsim_conv=drone_flight.ue4_airsim_factor,
                                            client=drone_flight.client)
        drone_flight.env_map.run(drone_flight.altitude, drone_flight.altitude_range, drone_flight.saved_vertices_filename,
                                 drone_flight.update_saved_vertices, drone_flight.plot2D, drone_flight.plot3D)

        drone_flight.obtain_start_goal()

        # Plot the start and goal points on the 3D grid
        if drone_flight.plot3D:
            drone_flight.env_map.plot_start_goal_3D_grid(drone_flight.start_grid, drone_flight.goal_grid)

        # Compute the computational time with the chosen navigation type and store it
        nav = GridNavigation(drone_flight.env_map, drone_flight.plot2D, drone_flight.plot3D)
        start_time_nav = time.time()
        _ = nav.navigation_A_star(drone_flight.start_grid, drone_flight.goal_grid, drone_flight.robot_radius)
        end_time_nav = time.time()
        PP_computation_time["A_star"].append(end_time_nav - start_time_nav)

        nav = GridNavigation(drone_flight.env_map, drone_flight.plot2D, drone_flight.plot3D)
        start_time_nav = time.time()
        _ = nav.navigate_wavefront(drone_flight.start_grid, drone_flight.goal_grid, drone_flight.robot_radius)
        end_time_nav = time.time()
        PP_computation_time["Wavefront"].append(end_time_nav - start_time_nav)

        nav = GridNavigation(drone_flight.env_map, drone_flight.plot2D, drone_flight.plot3D)
        start_time_nav = time.time()
        _ = nav.navigate_Voronoid(drone_flight.start_grid, drone_flight.goal_grid, drone_flight.robot_radius)
        end_time_nav = time.time()
        PP_computation_time["Voronoi"].append(end_time_nav - start_time_nav)

        nav = GridNavigation(drone_flight.env_map, drone_flight.plot2D, drone_flight.plot3D)
        start_time_nav = time.time()
        _ = nav.navigation_RRT_star(drone_flight.start_grid, drone_flight.goal_grid)
        end_time_nav = time.time()
        PP_computation_time["RRT*"].append(end_time_nav - start_time_nav)

        nav = GridNavigation(drone_flight.env_map, drone_flight.plot2D, drone_flight.plot3D)
        start_time_nav = time.time()
        _ = nav.navigation_PRM(drone_flight.start_grid, drone_flight.goal_grid, drone_flight.robot_radius)
        end_time_nav = time.time()
        PP_computation_time["PRM"].append(end_time_nav - start_time_nav)
else:  # it has already been obtained and it will be loaded next
    with open('../PP_computation_time.pickle', 'rb') as infile:
        PP_computation_time = pickle.load(infile)

# Obtaining the name of the algorithms considered
keys = PP_computation_time.keys()
pivot = PP_computation_time["A_star"]  # the algorithm class that we want to use as baseline for comparison
PP_computation_time_relative = {}
fig_num = 1
for key in keys:
    if key != "A_star":
        PP_computation_time_relative[key] = [(PP_computation_time[key][i] - pivot[i]) / pivot[i]
                                             for i in range(len(pivot))]
        mean = statistics.mean(PP_computation_time_relative[key])
        std = statistics.stdev(PP_computation_time_relative[key])

        # Plot the results in the form of a histogram, one for each navigation method
        plt.subplot(2, 2, fig_num)
        n, bins, patches = plt.hist(x=PP_computation_time_relative[key], bins='auto', color="#1f77b4",
                                    alpha=0.9, rwidth=0.85)
        plt.grid(axis='y', alpha=0.75)
        if key == 'RRT_star':
            key = 'RRT*'
        elif key == 'wavefront':
            key = 'Wavefront'
        plt.xlabel('m(' + key + ')')
        plt.ylabel('Frequency')
        matplotlib.rc('xtick')
        matplotlib.rc('ytick')
        # plt.title('Relative computational time wrt. A*')  # commented out for the paper figure generation
        print_text = r"$\mu$ = " + str(round(mean, 2)) + "\n" + r"$\sigma$ = " + str(round(std, 2))
        maxfreq = n.max()
        plt.text(bins[int(len(bins)*0.6)], (np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)*0.75,
                 print_text)

        # Set a clean upper y-axis limit.
        plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)

        fig_num += 1
    plt.tight_layout()
