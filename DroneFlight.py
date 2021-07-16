import airsim
import numpy as np
import time
import sys
import random
from datetime import datetime
import keyboard
from icecream import ic

from OccupancyMap import OccupancyMap
from GridNavigation import GridNavigation
from DroneSensors import DroneSensors
from utils import compute_distance_points
from user_input import load_user_input
from FailureFactory import FailureFactory


# TODO: take into account measurement time when considering the measurement frequency


class DroneFlight:
    """
    Class which carries out the complete flight from a single drone. From the generation of the map with OccupancyMap,
    to the obstacle avoidance with GridNavigation, to the collection of data with DroneSensors. It incorporates all
    the methods in order to make a single flight successful.
    """

    def __init__(self, altitude_m, altitude_range_m, cell_size_m, ue4_airsim_factor, robot_radius_m, sensors,
                 camera_info, sample_rates, min_flight_distance_m=30, max_flight_distance_m=200,
                 saved_vertices_filename='object_points', update_saved_vertices=False, plot2D=False, plot3D=True,
                 vehicle_name='', vehicle_start_position=None, smooth=True, failure_types=None, activate_take_off=True):
        if vehicle_start_position is None:
            vehicle_start_position = (0, 0, 0)
        self.altitude_m = altitude_m
        self.altitude_range_m = altitude_range_m
        self.cell_size_m = cell_size_m
        self.min_flight_distance_m = min_flight_distance_m
        self.max_flight_distance_m = max_flight_distance_m
        self.saved_vertices_filename = saved_vertices_filename
        self.update_saved_vertices = update_saved_vertices
        self.plot2D = plot2D
        self.plot3D = plot3D
        self.vehicle_name = vehicle_name
        self.vehicle_start_position = vehicle_start_position
        self.smooth = smooth

        self.start_grid = None
        self.goal_grid = None
        self.start_world = None
        self.goal_world = None
        self.path = None

        self.ue4_airsim_factor = ue4_airsim_factor
        self.robot_radius_m = robot_radius_m
        self.robot_radius = self.robot_radius_m / self.cell_size_m
        distances = [altitude_m, altitude_range_m, cell_size_m]
        altitude_flags = [True, False, False]
        self.altitude, self.altitude_range, self.cell_size = self.distances_to_ue4(distances, altitude_flags)

        self.client = None
        self.connect_airsim()

        self.env_map = None

        self.sensors = sensors
        self.camera_info = camera_info
        self.sample_rates = sample_rates
        self.sensors = DroneSensors(self.client, self.sensors, self.camera_info,
                                    self.sample_rates, vehicle_name=self.vehicle_name)

        self.failure_types = failure_types
        self.failure_factory = FailureFactory(self.client, self.failure_types, vehicle_name=self.vehicle_name)

        self.activate_take_off = activate_take_off

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
        print(self.start_grid, self.goal_grid)

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
        if navigation_type == "A_star":
            path = nav.navigation_A_star(self.start_grid, self.goal_grid, self.robot_radius)
        elif navigation_type == "wavefront":
            path = nav.navigate_wavefront(self.start_grid, self.goal_grid)
        elif navigation_type == "Voronoid":
            path = nav.navigate_Voronoid(self.start_grid, self.goal_grid, self.robot_radius)
        elif navigation_type == "RRT_star":
            path = nav.navigation_RRT_star(self.start_grid, self.goal_grid)
        elif navigation_type == "PRM":
            path = nav.navigation_PRM(self.start_grid, self.goal_grid, self.robot_radius)
        else:
            raise ValueError("Navigation type does not exist.")

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
                    success = not collision
                except:
                    success = False
                if not success:
                    path = path_or
                    print('Smoothening with a reduction of ' + str(reduction) + ' did not succeed.')
                    reduction += 0.05
                    reduction = np.round(reduction, 2)
                    if reduction > 1:
                        print('Smoothening did not succeed.')

        # Translate the path to AirSim coordinates and save the AirSim coordinates of the start and goal locations
        self.path = self.env_map.translate_path_lst_to_Vector3r(
            self.positions_world_to_drone_frame(self.env_map.translate_path_to_world_coord(path, self.altitude)))
        self.start_world = self.position_world_to_drone_frame(
            self.env_map.translate_point_to_world_coord(self.start_grid, 0))
        self.goal_world = self.position_world_to_drone_frame(
            self.env_map.translate_point_to_world_coord(self.goal_grid, 0))

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

        # In the case that it is not desired to carry out the take-off
        if not self.activate_take_off:
            # Check that API control is enabled
            if not self.client.isApiControlEnabled(vehicle_name=self.vehicle_name):
                self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
            # Start up the drone
            print("arming the drone...")
            self.client.armDisarm(True, vehicle_name=self.vehicle_name)

            # Set up the desired altitude and start hovering the drone in place
            pose.position.z_val = self.altitude_m
            self.client.moveToZAsync(self.altitude_m, 0.1, vehicle_name=self.vehicle_name)
            time.sleep(0.5)

        # Move vehicle to desired position
        self.client.simSetVehiclePose(pose, True, vehicle_name=self.vehicle_name)

    def take_off(self):
        """
        Method that arms the drone and makes it take-off
        :return:
        """
        # Start up the drone
        print("arming the drone...")
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)

        # Check proper take-off
        self.check_take_off()

        # AirSim uses NED coordinates so negative axis is up.
        print("make sure we are hovering at " + str(-self.altitude_m) + " meters...")
        self.client.moveToZAsync(self.altitude_m, 1, vehicle_name=self.vehicle_name).join()

    def check_take_off(self):
        """
        Method that checks that the drone has actually taken off. If not, an error is raised.
        :return:
        """
        if not self.client.isApiControlEnabled(vehicle_name=self.vehicle_name):
            self.client.enableApiControl(True, vehicle_name=self.vehicle_name)

        state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
        if state.landed_state == airsim.LandedState.Landed:
            print("taking off...")
            time.sleep(1)
            self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        else:
            self.client.hoverAsync(vehicle_name=self.vehicle_name).join()

        print("Waiting 2 seconds", self.vehicle_name)
        time.sleep(2)
        print("Done 2 seconds", self.vehicle_name)

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
        print("flying on path...")
        # self.client.moveOnPathAsync(self.path, 12, 120, airsim.DrivetrainType.ForwardOnly,
        #                             airsim.YawMode(False, 0), 20, 1)
        self.client.moveOnPathAsync(self.path, 12, 120, airsim.DrivetrainType.ForwardOnly,
                                    airsim.YawMode(False, 0), -1, 0, vehicle_name=self.vehicle_name)

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
            elif z_val > -3 * self.altitude_m:
                message = "Fly away"
                ic(message)
                return 0, distance, 3
            # Check whether the drone is on the ground and the collision has not been registered = very smooth landing
            elif z_val < 0.75:
                message = "Collision ground"
                ic(message)
                return 0, distance, 2

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
        self.sensors.start_signal_sensors_data_storage()

        not_arrived = 1
        collision_type = 0
        failed = 0
        # While the drone has not reached destination or collided
        while not_arrived:
            self.sensors.store_sensors_data()

            # Print the timestamp
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            print("Current Time =", current_time)

            # Obtain the distance to destination
            not_arrived, distance, collision_type = self.check_goal_arrival(failed)
            failed = self.failure_factory.execute_failures(distance)

            # Manual break in the collection of data
            if keyboard.is_pressed('K'):
                print('The letter K has been pressed.')
                break

        # Once the drone has arrived to its destination, the sensor and failure data is stored in their respective files
        self.failure_factory.write_to_file(collision_type, self.sensors.folder_name)
        self.sensors.write_to_file()

    def reset(self, reset_failures_airsim: bool = False):
        """
        Method that resets the state of the drone in order to carry out a new iteration. For that purpose, it resets the
        client, as well as returning the damaged coefficients to one.
        :return:
        """
        self.client.reset(reset_failures_airsim)
        self.failure_factory.reset()

    def run(self, navigation_type="A_star", start_point=None, goal_point=None, min_h=None, max_h=None,
            activate_reset=True):
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
        :return:
        """
        if min_h is not None and max_h is not None:
            h = -random.randint(min_h, max_h)
            self.extract_occupancy_map(h)
            ic(h)
        else:
            self.extract_occupancy_map()

        self.navigate_drone_grid(navigation_type=navigation_type, start_point=start_point, goal_point=goal_point)
        self.teleport_drone_start()
        time.sleep(2)
        if self.activate_take_off:
            self.take_off()
            time.sleep(2)
        self.select_failure()
        self.fly_trajectory()
        self.obtain_sensor_data()
        if activate_reset:
            self.reset(True)
        time.sleep(2)


if __name__ == "__main__":
    # User input
    args = load_user_input()

    # Fly drone
    drone_flight = DroneFlight(args.altitude_m, args.altitude_range_m, args.cell_size_m,
                               args.ue4_airsim_conversion_units, args.robot_radius, args.sensors_lst, args.cameras_info,
                               args.sample_rate, min_flight_distance_m=args.min_flight_distance_m,
                               saved_vertices_filename=args.saved_vertices_filename,
                               update_saved_vertices=args.update_saved_vertices, plot2D=args.plot2D, plot3D=args.plot3D,
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
import airsim
import time
client = airsim.MultirotorClient()
client.armDisarm(True)
client.simGetImages([airsim.ImageRequest("0", 0)])
client.setBarometerActivation(True, 56, False)
time.sleep(10)
potato = client.getBarometerStoredDataVec()
client.cleanBarometerStoredData()
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


# 22.4
# 15.61
# 19.43
# [times[i+1] - times[i] for i in range(len(times)-1)]
# times = [int(i[:-4]) for i in os.listdir("E:\Master_project\Simulator\AirSim_simulator\AirSim\PythonClient\multirotor\Occupancy_grid\Sensor_data\20210715-163858_1\front")[:-1]]