import sys
import json
import random
import time
import pickle
import statistics
import matplotlib
import numpy as np
from icecream import ic
import matplotlib.pyplot as plt

from user_input import load_user_input
from Drone_flight.DroneFlight import DroneFlight
from Environment_extraction.OccupancyMap import OccupancyMap
from Drone_grid_navigation.GridNavigation import GridNavigation

recompute_statistics = False

if recompute_statistics:
    args = load_user_input()

    # Debugging with icecream
    gettrace = getattr(sys, 'gettrace', None)
    if gettrace() and not args.PSO_tuning_switch:
        ic.enable()
    else:
        ic.disable()

    # Retrieving the start position of the drone
    location_json_file = "C:\\Users\jialv\OneDrive\Documentos\AirSim\settings.json"
    with open(location_json_file) as f:
        data = json.load(f)
    vehicle_name = list(data['Vehicles'].keys())[0]
    coord = data['Vehicles'][vehicle_name]
    start_coord = (coord['X'], coord['Y'], coord['Z'])

    # Obtaining the clock speed information
    clock_speed = data['ClockSpeed']
    sample_rates = {key: value * clock_speed for key, value in {}.items()}

    # Create a Drone
    # Creating a drone flight to exploit some of the functions
    drone_flight = DroneFlight(altitude_m=-1, altitude_range_m=3, cell_size_m=3,
                               ue4_airsim_factor=100, robot_radius_m=9,
                               sensors=[],
                               camera_info={},
                               sample_rates=sample_rates, clock_speed=clock_speed,
                               min_flight_distance_m=30,
                               max_flight_distance_m=200,
                               saved_vertices_filename='object_points',
                               update_saved_vertices=False, plot2D=False,
                               plot3D=False,
                               controller_tuning_switch=False,
                               data_gather_types=[],
                               plotting_controller_signals_aeo=[],
                               plotting_controller_signals=[],
                               vehicle_name=vehicle_name,
                               vehicle_start_position=start_coord, smooth=True,
                               failure_types=[],
                               activate_take_off=False)

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
else:
    with open('../PP_computation_time.pickle', 'rb') as infile:
        PP_computation_time = pickle.load(infile)

keys = PP_computation_time.keys()
pivot = PP_computation_time["A_star"]
PP_computation_time_relative = {}
fig_num = 1
for key in keys:
    if key != "A_star":
        PP_computation_time_relative[key] = [(PP_computation_time[key][i] - pivot[i]) / pivot[i] for i in range(len(pivot))]
        mean = statistics.mean(PP_computation_time_relative[key])
        std = statistics.stdev(PP_computation_time_relative[key])
        # An "interface" to matplotlib.axes.Axes.hist() method
        # plt.figure(fig_num)
        plt.subplot(2, 2, fig_num)
        n, bins, patches = plt.hist(x=PP_computation_time_relative[key], bins='auto', color='#0504aa',
                                    alpha=0.9, rwidth=0.85)
        plt.grid(axis='y', alpha=0.75)
        if key == 'RRT_star':
            key = 'RRT*'
        elif key == 'wavefront':
            key = 'Wavefront'
        # plt.xlabel('(t$_{' + key + '}$-t$_{A*}$)/t$_{A*}$', fontsize=13)
        plt.xlabel('m(' + key + ')', fontsize=13)
        plt.ylabel('Frequency', fontsize=13)
        matplotlib.rc('xtick', labelsize=12)
        matplotlib.rc('ytick', labelsize=12)
        # plt.title('Relative computational time wrt. A*')
        print_text = r"$\mu$ = " + str(round(mean, 2)) + "\n" + r"$\sigma$ = " + str(round(std, 2))
        maxfreq = n.max()
        plt.text(bins[int(len(bins)*0.6)], (np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)*0.75, print_text, fontsize=12)
        # Set a clean upper y-axis limit.
        plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)

        fig_num += 1
    plt.tight_layout()
