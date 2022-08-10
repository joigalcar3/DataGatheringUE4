import airsim
import numpy as np
from math import atan2, pi, sin, cos, degrees
import sys
import random
from datetime import datetime
import keyboard
from icecream import ic
import time

from Environment_extraction.OccupancyMap import OccupancyMap
from Drone_grid_navigation.GridNavigation import GridNavigation
from Drone_flight.Data_gathering.DroneSensors import DroneSensors
from utils import compute_distance_points
from user_input import load_user_input
from Drone_flight.Failure_injection.FailureFactory import FailureFactory
from Drone_flight.ControllerTuning import ControllerTuning


# TODO: take into account measurement time when considering the measurement frequency


class DroneFlight:
    """
    Class which carries out the complete flight from a single drone. From the generation of the map with OccupancyMap,
    to the obstacle avoidance with GridNavigation, to the collection of data with DroneSensors. It incorporates all
    the methods in order to make a single flight successful.
    """

    def __init__(self, user_input, sample_rates, clock_speed=1, vehicle_name='', vehicle_start_position=None):
        if vehicle_start_position is None:
            vehicle_start_position = (0, 0, 0)
        self.altitude_m = -user_input.altitude_m
        self.altitude_range_m = user_input.altitude_range_m
        self.cell_size_m = user_input.cell_size_m
        self.min_flight_distance_m = user_input.min_flight_distance_m
        self.max_flight_distance_m = user_input.max_flight_distance_m
        self.saved_vertices_filename = user_input.saved_vertices_filename
        self.update_saved_vertices = user_input.update_saved_vertices
        self.plot2D = user_input.plot2D
        self.plot3D = user_input.plot3D
        self.vehicle_name = vehicle_name
        self.vehicle_start_position = vehicle_start_position
        self.smooth = user_input.smooth

        self.start_grid = None
        self.goal_grid = None
        self.start_world = None
        self.goal_world = None
        self.path = None
        self.heading_start = None

        self.ue4_airsim_factor = user_input.ue4_airsim_conversion_units
        self.robot_radius_m = user_input.robot_radius
        self.robot_radius = self.robot_radius_m / self.cell_size_m
        distances = [self.altitude_m, self.altitude_range_m, self.cell_size_m]
        altitude_flags = [True, False, False]
        self.altitude, self.altitude_range, self.cell_size = self.distances_to_ue4(distances, altitude_flags)

        self.client = None
        self.connect_airsim()

        self.env_map = None

        self.sensors = user_input.sensors_lst
        self.clock_speed = clock_speed
        self.sensors = DroneSensors(user_input, self.client, self.sensors, sample_rates, vehicle_name=self.vehicle_name)

        self.failure_factory = FailureFactory(user_input, self.client, self.clock_speed, vehicle_name=self.vehicle_name)
        self.collision_type = -1

        self.activate_take_off = user_input.activate_take_off

        # Controller tuning
        self.controller_tuning_switch = user_input.controller_tuning_switch
        self.controller_tuning = ControllerTuning(user_input, self.client, self.controller_tuning_switch,
                                                  vehicle_name=self.vehicle_name)

    def distances_to_ue4(self, distances, altitude_flags):
        """
        Converts a list of distances from AirSim metres to UE4 units
        :param distances: list of distances to convert to UE4 units
        :param altitude_flags: whether each of the provided units are altitudes. If that is the case, it must be taken
        into account that negative UE4 altitude units are positive altitudes
        :return ue4_distances: list with converted units
        """
        ue4_distances = []
        for i in range(len(distances)):
            ue4_distance = self.distance_to_ue4(distances[i], altitude_flags[i])
            ue4_distances.append(ue4_distance)
        return ue4_distances

    def distance_to_ue4(self, distance, altitude=False):
        """
        Converts a distance to UE4 units
        :param distance: distance in AirSim units to convert to UE4 units
        :param altitude: whether the provided distance is an altitude
        :return ue4_distance: distance in UE4 units
        """
        # Inputs converted to UE4 unit system
        factor = 1
        if altitude:
            factor = -1
        ue4_distance = factor * int(distance * self.ue4_airsim_factor)  # [UE4_units]
        return ue4_distance

    def positions_world_to_drone_frame(self, positions):
        """
        Method which takes a list of points and feeds them to the world2drone frame transformer.
        :param positions: list of points in the world reference frame
        :return: list of points in the drone reference frame
        """
        transformed_positions = []
        for position in positions:
            transformed_positions.append(self.position_world_to_drone_frame(position))
        return transformed_positions

    def position_world_to_drone_frame(self, position):
        """
        It has been observed that if the drone is not initialised at [0,0,0], then there is a shift between the world
        and the drone coordinate system. This method aims at correcting that error, since some functions such as
        self.client.simSetVehiclePose() require drone frame coordinates.
        :param position: the position in the world reference frame
        :return: position in the drone reference frame
        """
        drone_frame_position_x = round(position[0] - self.vehicle_start_position[0], 2)
        drone_frame_position_y = round(position[1] - self.vehicle_start_position[1], 2)
        if len(position) == 3:
            drone_frame_position_z = round(position[2] - self.vehicle_start_position[2], 2)
            new_pos = (drone_frame_position_x, drone_frame_position_y, drone_frame_position_z)
        elif len(position) == 2:
            new_pos = (drone_frame_position_x, drone_frame_position_y)
        else:
            message = 'The function position_world_to_drone_frame only accepts positions in 2d or 3d.'
            raise ValueError(message)
        return new_pos

    def connect_airsim(self):
        """
        Method to establish the connection with the AirSim Multirotor
        :return:
        """
        # Connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.vehicle_name)
        self.client.armDisarm(False, self.vehicle_name)

    def extract_occupancy_map(self, altitude_m=None):
        """
        Method which extracts the 2D Occupancy grid using the provided altitude for slicing the point cloud
        :param altitude_m: altitude provided in meters
        :return:
        """
        if altitude_m is not None:
            self.altitude_m = altitude_m
            self.altitude = self.distance_to_ue4(altitude_m, True)

        # Extract occupancy grid
        self.env_map = OccupancyMap(cell_size=self.cell_size, ue4_airsim_conv=self.ue4_airsim_factor,
                                    client=self.client)
        self.env_map.run(self.altitude, self.altitude_range, self.saved_vertices_filename, self.update_saved_vertices,
                         self.plot2D, self.plot3D)

    def obtain_start_goal(self):
        """
        Method to obtain a random start and goal location within the constraints that they need to be separated with
        a minimum distance and that they need to be separated from the obstacles with a minimum distance of robot_radius
        :return:
        """
        x_dim = self.env_map.extent_x - 1
        y_dim = self.env_map.extent_y - 1

        # Min distance in grid coordinates
        min_distance_grid = np.ceil(self.min_flight_distance_m / self.cell_size_m)
        max_distance_grid = np.ceil(self.max_flight_distance_m / self.cell_size_m)
        self.start_grid = (random.randint(0, x_dim), random.randint(0, y_dim))
        self.goal_grid = self.start_grid

        # Generate new points as long as the points are generated on top of an obstacle or the distance between them is
        # below the defined threshold.
        while compute_distance_points(self.start_grid, self.goal_grid) < min_distance_grid or \
                compute_distance_points(self.start_grid, self.goal_grid) > max_distance_grid or \
                self.env_map.check_obstacle(self.goal_grid, self.robot_radius) or \
                self.env_map.check_obstacle(self.start_grid, self.robot_radius):
            self.start_grid = (random.randint(0, x_dim), random.randint(0, y_dim))
            self.goal_grid = (random.randint(0, x_dim), random.randint(0, y_dim))
        ic(self.start_grid, self.goal_grid)

    def navigate_drone_grid(self, navigation_type="A_star", start_point=None, goal_point=None):
        """
        Method which obtains the path of grid points to follow in order to avoid the obstacles. First, the start and
        goal points are created if they have not been provided, then the navigation is computed with the method
        specified and finally, the path is smoothened with the B-spline if required.
        :param navigation_type: type of navigation chosen. The types are those outlined in GridNavigation.py
        :param smooth: whether the path needs to be smoothened with the B-spline
        :param start_point: the starting point of the flight
        :param goal_point: the final point of the flight
        :return:
        """
        # If the start and goal points are not provided or the ones provided are on an obstacle, then they are generated
        if (start_point is None and goal_point is None) or \
                self.env_map.check_obstacle(start_point, self.robot_radius) or \
                self.env_map.check_obstacle(goal_point, self.robot_radius):
            self.obtain_start_goal()
        else:
            self.start_grid = start_point
            self.goal_grid = goal_point

        # Plot the start and goal points on the 3D grid
        if self.plot3D:
            self.env_map.plot_start_goal_3D_grid(self.start_grid, self.goal_grid)

        # Compute the path with the chosen navigation type
        nav = GridNavigation(self.env_map, self.plot2D, self.plot3D)
        start_time_nav = time.time()
        if navigation_type == "A_star":
            path = nav.navigation_A_star(self.start_grid, self.goal_grid, self.robot_radius)
        elif navigation_type == "wavefront":
            path = nav.navigate_wavefront(self.start_grid, self.goal_grid, self.robot_radius)
        elif navigation_type == "Voronoid":
            path = nav.navigate_Voronoid(self.start_grid, self.goal_grid, self.robot_radius)
        elif navigation_type == "RRT_star":
            path = nav.navigation_RRT_star(self.start_grid, self.goal_grid)
        elif navigation_type == "PRM":
            path = nav.navigation_PRM(self.start_grid, self.goal_grid, self.robot_radius)
        else:
            raise ValueError("Navigation type does not exist.")
        end_time_nav = time.time()
        ic(end_time_nav - start_time_nav)
        # Smoothen the path
        if self.smooth:
            path_or = path.copy()
            success = False
            reduction = 0.05
            # Repeat the smoothening as long as the reduction is less than 1 (a smoothening is performed) or there is
            # a collision along the smoothened path
            while not success and reduction <= 1:
                try:
                    path, collision = nav.smooth_B_spline(path_or, reduction=reduction)
                    if not collision:
                        path, collision = nav.smooth_cubic_spline(path)
                    success = not collision
                except:
                    success = False
                if not success:
                    path = path_or
                    ic('Smoothening with a reduction of ' + str(reduction) + ' did not succeed.')
                    reduction += 0.05
                    reduction = np.round(reduction, 2)
                    if reduction > 1:
                        ic('Smoothening did not succeed.')

        # Translate the path to AirSim coordinates and save the AirSim coordinates of the start and goal locations
        path_world_coord = self.env_map.translate_path_to_world_coord(path, self.altitude)
        path_drone_coord = self.positions_world_to_drone_frame(path_world_coord)
        self.compute_starting_heading(path_drone_coord)
        self.path = self.env_map.translate_path_lst_to_Vector3r(path_drone_coord)
        self.start_world = self.position_world_to_drone_frame(
            self.env_map.translate_point_to_world_coord(self.start_grid, 0))
        self.goal_world = self.position_world_to_drone_frame(
            self.env_map.translate_point_to_world_coord(self.goal_grid, 0))

    def compute_starting_heading(self, path):
        start_point = path[0]
        second_point = path[1]
        heading_vector = [second_point[0]-start_point[0], second_point[1]-start_point[1]]
        self.heading_start = pi/2 - atan2(heading_vector[0], heading_vector[1])
        if self.heading_start > pi:
            self.heading_start = -(2*pi-self.heading_start)

    def teleport_drone_start(self):
        """
        Method that teleports the drone to the start location
        :return:
        """
        # Initialize sensors
        self.sensors.initialize_sensors()

        # Teleport drone to the start position
        pose = self.client.simGetVehiclePose(vehicle_name=self.vehicle_name)
        pose.position.x_val = self.start_world[0]
        pose.position.y_val = self.start_world[1]
        pose.orientation.x_val = 0
        pose.orientation.y_val = 0
        pose.orientation.z_val = sin(self.heading_start/2)
        pose.orientation.w_val = cos(self.heading_start/2)

        # In the case that it is not desired to carry out the take-off
        if not self.activate_take_off:
            # Check that API control is enabled
            if not self.client.isApiControlEnabled(vehicle_name=self.vehicle_name):
                self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
            # Start up the drone
            ic("arming the drone...")
            self.client.armDisarm(True, vehicle_name=self.vehicle_name)

            # Set up the desired altitude and start hovering the drone in place
            pose.position.z_val = self.altitude_m

        # Set desired yaw angle once it has been teleported
        self.client.setTeleportYawRef(degrees(self.heading_start))

        # Move vehicle to desired position
        self.client.simSetVehiclePose(pose, True, vehicle_name=self.vehicle_name)

    def take_off(self):
        """
        Method that arms the drone and makes it take-off
        :return:
        """
        # Start up the drone
        ic("arming the drone...")
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)

        # Check proper take-off
        self.check_take_off()

        # AirSim uses NED coordinates so negative axis is up.
        ic("make sure we are hovering at " + str(-self.altitude_m) + " meters...")
        self.client.moveToZAsync(self.altitude_m, 1, vehicle_name=self.vehicle_name).join()

    def check_take_off(self):
        """
        Method that checks that the drone has actually taken off. If not, an error is raised.
        :return:
        """
        # Making sure that Api control has been enabled
        if not self.client.isApiControlEnabled(vehicle_name=self.vehicle_name):
            self.client.enableApiControl(True, vehicle_name=self.vehicle_name)

        # Check if the drone has taken-off. If not, take-off, otherwise hover.
        state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
        if state.landed_state == airsim.LandedState.Landed:
            ic("taking off...")
            time.sleep(1)
            self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        else:
            self.client.hoverAsync(vehicle_name=self.vehicle_name).join()

        ic("Waiting 2 seconds", self.vehicle_name)
        time.sleep(2)
        ic("Done 2 seconds", self.vehicle_name)

        # If not able to take-off, exit and report error
        state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
        if state.landed_state == airsim.LandedState.Landed:
            print("take off failed...", self.vehicle_name)
            sys.exit(1)

    def select_failure(self):
        """
        Function that selects the failure type that will take place during the flight and the moment in time.
        :return:
        """
        # Obtain start location
        start_x = self.start_world[0]
        start_y = self.start_world[1]

        # Obtain goal location
        goal_x = self.goal_world[0]
        goal_y = self.goal_world[1]

        # Compute the distance between both locations
        distance = np.sqrt((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2)

        # Choose the failure and the location where it should take place
        self.failure_factory.failure_selection(distance)

    def fly_trajectory(self):
        """
        Method that flies the drone along the computed trajectory
        :return:
        """
        # see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo
        # this method is async and we are not waiting for the result since we are passing timeout_sec=0.
        ic("flying on path...")
        # self.client.moveOnPathAsync(self.path, 12, 120, airsim.DrivetrainType.ForwardOnly,
        #                             airsim.YawMode(False, 0), 20, 1)
        self.client.moveOnPathAsync(self.path, 1, 10000, airsim.DrivetrainType.ForwardOnly,
                                    airsim.YawMode(False, 0), 2, 1, vehicle_name=self.vehicle_name)

    def check_goal_arrival(self, failed):
        """
        Method that checks whether the drone has arrived to its destination. It is considered that the drone has arrived
        to its destination if the distance between its position and the goal is less than 2 AirSim metres.
        :return: whether the drone has arrived to its goal
        """
        # Obtain drone and goal locations
        drone_location = self.client.simGetVehiclePose(vehicle_name=self.vehicle_name)
        real_x, real_y, goal_x, goal_y = drone_location.position.x_val, drone_location.position.y_val, \
                                         self.goal_world[0], self.goal_world[1]
        # Compute the distance between both locations
        distance = np.sqrt((goal_x - real_x) ** 2 + (goal_y - real_y) ** 2)
        message = "".join([self.vehicle_name, '. Goal: (', str(goal_x), ',', str(goal_y), '). Drone location: (',
                           str(real_x), ',', str(real_y), '). Distance: ', str(distance), '. Altitude: ',
                           str(-self.client.getMultirotorState(vehicle_name=self.vehicle_name).kinematics_estimated.
                               position.z_val), '. Desired altitude: ', str(-self.altitude_m), '.'])
        ic(message)

        # Check whether the drone has collided
        if failed:
            collision_info = self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
            z_val = -self.client.getMultirotorState(vehicle_name=self.vehicle_name).kinematics_estimated.position.z_val
            current_time = self.client.getMultirotorState(vehicle_name=self.vehicle_name).timestamp/1e9
            collided = collision_info.has_collided
            ic(collided)
            if collided:
                collision_object = collision_info.object_name
                # Check if it has collided against the ground or an obstacle
                if collision_object[0:6] == 'Ground':
                    message = "Collision ground"
                    ic(message)
                    return 0, distance, 2
                else:
                    message = "Collision obstacle"
                    ic(message)
                    return 0, distance, 1
            # Check whether the drone flies off into the sky
            elif z_val > -2 * self.altitude_m:
                message = "Fly away"
                ic(message)
                return 0, distance, 3
            # Check whether the drone is on the ground and the collision has not been registered = very smooth landing
            elif z_val < 0.75:
                message = "Collision ground"
                ic(message)
                return 0, distance, 2
            elif current_time-self.failure_factory.failure_timestamp >= 2:
                message = '2 seconds passed since failure'
                ic(message)
                return 0, distance, 4

        # Check whether the distance is less than 2 metres
        if distance < 2:
            message = "Reached destination"
            ic(message)
            return 0, distance, 0

        return 1, distance, 0

    def obtain_sensor_data(self):
        """
        Method that retrieves the data from the sensors given the sample rate and executes the failure.
        :return:
        """
        # Starting to store the data from the sensors and for tuning the controller
        self.sensors.start_signal_sensors_data_storage()
        self.controller_tuning.initialize_data_gathering()
        # failed = int(self.controller_tuning_switch)
        failed = 0

        not_arrived = 1
        self.collision_type = 0
        start_time = time.time()
        # While the drone has not reached destination or collided
        while not_arrived:
            # When tuning the controller, storing the sensor data is not required
            if not self.controller_tuning_switch:
                self.sensors.store_sensors_data()

            # Print the timestamp
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            ic("Current Time =", current_time)

            # Obtain the distance to destination
            not_arrived, distance, self.collision_type = self.check_goal_arrival(failed)
            # When tuning the controller, executing failures is not required
            # if not self.controller_tuning_switch:
            failed = self.failure_factory.execute_failures(distance)

            # Manual break in the collection of data
            if keyboard.is_pressed('K'):
                print('The letter K has been pressed.')
                break

            # When tuning the controller (also when not tuning), the maximum flight time is limited --> 40 sec
            end_time = time.time()
            if (end_time-start_time) > 40/self.clock_speed:
                not_arrived, distance, self.collision_type = [0, 100, 5]

        # Once the drone has arrived to its destination, the sensor and failure data is stored in their respective files
        # When the controller is being tuned, failure and sensor information is not collected
        if not self.controller_tuning_switch:
            self.failure_factory.write_to_file(self.collision_type, self.sensors.folder_name)
            self.sensors.write_to_file()

    def reset(self, reset_failures_airsim: bool = False):
        """
        Method that resets the state of the drone in order to carry out a new iteration. For that purpose, it resets the
        client, as well as returning the damaged coefficients to one.
        :return:
        """
        self.failure_factory.reset(reset_failures_airsim)
        self.client.reset()
        self.sensors.restart_sensors()
        self.controller_tuning.reset()

    def run(self, navigation_type="A_star", start_point=None, goal_point=None, min_h=None, max_h=None,
            activate_reset=True, activate_map_extraction=True):
        """
        Method that carries out the complete flight of a drone. First, the occupancy map is extracted and the navigation
        of the drone is computed. Then, the drone is teleported to the start, the drone takes-off and flies the
        trajectory. Along the way, the sensor data is collected. Once concluded, the drone is brough back to the start
        :param navigation_type: the type of navigation used
        :param start_point: the start location for the flight
        :param goal_point: the goal location of the flight
        :param max_h: maximum altitude considered for the flight
        :param min_h: minimum altitude considered for the flight
        :param activate_reset: whether the airsim client needs to be reseted after the run
        :param activate_map_extraction: whether the map should be extracted. When carrying out multiple runs, extracting
        the environment every time is very expensive. Hence, the altitude could be fixed for multiple runs such that
        the map remains constant. When the altitude is maintained constant, the map does not have to be extracted.
        :return:
        """
        # Obtain occupancy map from the AirSim environment
        if activate_map_extraction:
            if self.altitude_m > 0:
                h = -random.randint(min_h, max_h)
                self.extract_occupancy_map(h)
                ic(h)
            else:
                self.extract_occupancy_map()

        # Navigate the drone within occupancy map and translate to AirSim path points
        self.navigate_drone_grid(navigation_type=navigation_type, start_point=start_point, goal_point=goal_point)

        # Teleport drone to start
        self.teleport_drone_start()
        time.sleep(2)
        if self.activate_take_off:
            self.take_off()
            time.sleep(2)

        # Choose drone failure
        self.select_failure()

        # Fly drone trajectory
        self.fly_trajectory()

        # Collect all sensor data
        self.obtain_sensor_data()

        # Execute methods used for controller tuning
        total_error = self.controller_tuning.tuning_cost_function(self.collision_type, self.path)
        self.controller_tuning.scope_plotting_signals()

        # self.controller_tuning.scope_plotting_signals(True,
        #                                               "C:\\Users\\jialv\\OneDrive\\2020-2021\\Thesis project\\3_Execution_phase\\Simulator_images\\Debugging\\Positive_zero_correct_V_and_float_and_filter")
        # self.controller_tuning.scope_plotting_signals(True,
        #                                               "C:\\Users\\jialv\\OneDrive\\2020-2021\\Thesis project\\3_Execution_phase\\Simulator_images\\Debugging\\Positive_zero_correct_V")
        # self.controller_tuning.scope_plotting_signals(True,
        #                                               "C:\\Users\\jialv\\OneDrive\\2020-2021\\Thesis project\\3_Execution_phase\\Simulator_images\\Debugging\\Negative_zero_correct_V_and_float_and_filter_and_posd")
        # Reset all the stored values from the flight
        if activate_reset:
            self.reset(True)
        return total_error


if __name__ == "__main__":
    # User input
    args = load_user_input()

    # Fly drone
    drone_flight = DroneFlight(args.altitude_m, args.altitude_range_m, args.cell_size_m,
                               args.ue4_airsim_conversion_units, args.robot_radius, args.sensors_lst, args.cameras_info,
                               args.sample_rate, min_flight_distance_m=args.min_flight_distance_m,
                               saved_vertices_filename=args.saved_vertices_filename,
                               update_saved_vertices=args.update_saved_vertices, plot2D=args.plot2D, plot3D=args.plot3D,
                               PID_tuning=args.PID_tuning,
                               failure_types=args.failure_types, activate_take_off=args.activate_take_off)
    drone_flight.run(navigation_type=args.navigation_type, start_point=args.start, goal_point=args.goal)

#
# import airsim
# import time
# client = airsim.MultirotorClient()
# client.setImuActivation(True, 2000, False)
# time.sleep(10)
# potato = client.getImuStoredDataVec()
# client.cleanImuStoredData()
#
# print(len(potato['timestamps']))
# print((potato['timestamps'][-1] - potato['timestamps'][0])/1e9)
# [(potato['timestamps'][i+1]-potato['timestamps'][i])/1e9 for i in range(len(potato['timestamps'])-1)]
# sum([(potato['timestamps'][i+1]-potato['timestamps'][i])/1e9 for i in range(len(potato['timestamps'])-1)])
#
#
#
#
#
# import airsim
# import time
# client = airsim.MultirotorClient()
# client.armDisarm(True)
# client.simGetImages([airsim.ImageRequest("0", 0)])
# client.setBarometerActivation(True, 56, False)
# time.sleep(10)
# potato = client.getBarometerStoredDataVec()
# client.cleanBarometerStoredData()
#
# print(len(potato['timestamps']))
# print((potato['timestamps'][-1] - potato['timestamps'][0])/1e9)
#
#
# import airsim
# import time
# client = airsim.MultirotorClient()
# client.setMagnetometerActivation(True, 56, False)
# time.sleep(10)
# potato = client.getMagnetometerStoredDataVec()
# client.cleanMagnetometerStoredData()
#
# print(len(potato['timestamps']))
# print((potato['timestamps'][-1] - potato['timestamps'][0])/1e9)
#
# import airsim
# import time
# client = airsim.MultirotorClient()
# client.setGpsActivation(True, 1000, False)
# time.sleep(10)
# potato = client.getGpsStoredDataVec()
# client.cleanGpsStoredData()
#
# print(len(potato['timestamps']))
# print((potato['timestamps'][-1] - potato['timestamps'][0])/1e9)



# import os
# times = [int(i[:-4]) for i in os.listdir("D:\AirSim simulator\AirSim\PythonClient\multirotor\Occupancy_grid\Sensor_data\20210715-163858_1\front")[:-1]]
# time_passed = [times[i+1] - times[i] for i in range(len(times)-1)]
# [abs(time_passed[i+1] - time_passed[i]) for i in range(len(time_passed)-1)]
#
#
# import os
# times = [int(i[15:-4]) for i in os.listdir("D:\AirSim simulator\AirSim\PythonClient\multirotor\Occupancy_grid\Sensor_data\Test\\2021-07-21-16-52-37\images")[:-1]]
# time_passed = [times[i+1] - times[i] for i in range(len(times)-1)]
# [abs(time_passed[i+1] - time_passed[i]) for i in range(len(time_passed)-1)]


# import time
# import airsim
# client = airsim.MultirotorClient()
# client.startRecording()
# time.sleep(10)
# client.stopRecording()
