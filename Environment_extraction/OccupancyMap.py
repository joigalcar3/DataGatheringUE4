#!/usr/bin/env python
"""
Provides the OccupancyMap class in charge of building, manipulating and visualizing the occupancy map used for the
navigation of the vehicle in the UE4 environment without colliding with obstacles.
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
import os
import airsim
import pickle
from math import sqrt
from icecream import ic
import matplotlib.pyplot as plt
from matplotlib.path import Path


from Plotter3D import *
from user_input import load_user_input
from utils import counter_clockwise_ordering, compute_distance_points, obtain_outer_edge


class OccupancyMap:
    """
    Class which provides all the tools for extracting the point cloud from the UE4 simulator, build the occupancy map
    and further manipulating it. Additionally, it provides 2D and 3D plots to visualize the occupancy map.
    """
    def __init__(self, folder="Environment_extraction//Point_files", extent_x=100, extent_y=100, cell_size=1,
                 ue4_airsim_conv=100, client=None):
        """
        Initializes the OccupancyMap class
        :param folder: location where the point cloud is stored or will be stored
        :param extent_x: default size of the occupancy map along the x-axis
        :param extent_y: default size of the occupancy map along the y-axis
        :param cell_size: the number of UE4 simulation distance units that represent the side of a square cell in the
        occupancy map
        :param ue4_airsim_conv: the conversion of distance units from the UE4 coordinate frame to the AirSim coordinate
        frame
        :param client: the AirSim client object
        """
        self.client = client
        self.folder = os.path.join(os.getcwd(), folder)
        isExist = os.path.exists(self.folder)
        if not isExist:
            os.makedirs(self.folder)

        self.filename = None
        self.ue4_airsim_conv = ue4_airsim_conv

        self.filtered_points = None
        self.filtered_points_array = None

        self.extent_x = extent_x
        self.extent_y = extent_y
        self.cell_size = cell_size
        self.limit_x_min = int((-0.5 * self.extent_x) * self.cell_size)    # Lowest x value in UE4 coordinate frame
        self.limit_y_min = int((-0.5 * self.extent_y) * self.cell_size)    # Lowest y value in UE4 coordinate frame
        self.limit_x_max = None   # Highest x value in UE4 coordinate frame
        self.limit_y_max = None   # Highest y value in UE4 coordinate frame
        self.grid = np.zeros([self.extent_x, self.extent_y], dtype=np.bool)  # occupancy map grid
        self.object_grid = np.zeros([self.extent_x, self.extent_y], dtype=np.bool)
        self.object_grid_coord = None
        self.object_grid_coord_all = None

        self.plotter = None

    def extract_obstacle_vertices(self, filename='objects_points', update_points=False):
        """
        Extract the vertices coordinates from the AirSim recursive data structure
        :param filename: name of file where to save the data
        :param update_points: if the points exist, whether they should be updated
        :return: None
        """
        self.filename = filename   # Name of the file where the point clouds are saved
        complete_filename = self.filename + '.p'   # Complete name with extension
        path_save = os.path.join(self.folder, complete_filename)

        # If the points should be updated or there is no saved point cloud
        if update_points or not os.path.isfile(path_save):
            point_cloud = self.client.simGetMeshPositionVertexBuffers()
            objects = {}
            for i_mesh in range(len(point_cloud)):   # For each of the objects in the data structure
                object_name = point_cloud[i_mesh].name
                points = np.reshape(np.array(point_cloud[i_mesh].vertices), [-1, 3])
                cut = np.max(point_cloud[i_mesh].indices) + 1
                objects[object_name] = points[:cut, :]   # Obtain only the points that are really part of the obstacle
            pickle.dump(objects, open(path_save, 'wb'))  # Store the point cloud

    def set_env_dims(self):
        """
        Obtain the maximum dimensions of the environment from the filtered points.
        :return: None
        """
        if self.filtered_points is None:
            self.project_points_altitude()

        # Iterating over the coordinates of all the assets in the UE4 environment
        object_names = self.filtered_points.keys()
        min_xy = 2 * [0]
        max_xy = 2 * [0]
        for name in object_names:
            local_min = [np.min(self.filtered_points[name][:, 0]), np.min(self.filtered_points[name][:, 1])]  # Minimum coordinates for current obstacle
            local_max = [np.max(self.filtered_points[name][:, 0]), np.max(self.filtered_points[name][:, 1])]  # Maximum coordinates for current obstacle
            min_xy = [min(value) for value in zip(min_xy, local_min)]   # Minimum value in the x and y directions [x_min, y_min]
            max_xy = [max(value) for value in zip(max_xy, local_max)]   # Maximum value in the x and y directions [x_max, y_max]

        self.extent_x = int(np.ceil((max_xy[0] - min_xy[0]) / self.cell_size))   # Number of cells in the x direction
        self.extent_y = int(np.ceil((max_xy[1] - min_xy[1]) / self.cell_size))   # Number of cells in the y direction
        self.limit_x_min = int(min_xy[0]) - (1 * (np.sign(min_xy[0]) == -1))     # Minimum x-coordinate in the UE4 reference frame
        self.limit_y_min = int(min_xy[1]) - (1 * (np.sign(min_xy[0]) == -1))     # Minimum y-coordinate in the UE4 reference frame
        ic("Real environment dimensions are: " + str(self.extent_x) + " x " + str(self.extent_y))

        # Update the grid objects
        self.grid = np.zeros([self.extent_x, self.extent_y], dtype=np.bool)
        self.object_grid = np.zeros([self.extent_x, self.extent_y], dtype=np.bool)

    def project_points_altitude(self, h=1100, delta_h=500):
        """
        Project to the x-y plane all the points at the sliced altitude
        :param h: altitude
        :param delta_h: range of altitudes where points can be found
        :return self.filtered_points: dictionary with the filtered point cloud for each object in the environment
        """
        # Locate point cloud file
        complete_filename = self.filename + '.p'
        path_save = os.path.join(self.folder, complete_filename)

        # Load the point cloud and remove the objects' points coordinates into the 2D plane
        space_points = pickle.load(open(path_save, 'rb'))
        object_names = space_points.keys()
        filtered_points = {}
        h_max = h + delta_h
        h_min = max(h - delta_h, 150)
        counter_points = 0
        for name in object_names:
            points = space_points[name]
            counter_points += len(points)
            point_filtered = points[(points[:, -1] <= h_max) &
                                    (points[:, -1] >= h_min)]   # Filter the points found within the range
            pf_projected = point_filtered[:, :-1]   # Remove the altitude component
            if pf_projected.size and name != 'externalcamera' and 'cinecameraactor' not in name:   # Only considered objects which have points in the point cloud slice
                filtered_points[name] = pf_projected  # Object filtered points
        self.filtered_points = filtered_points
        ic("Total number of 3D points: " + str(counter_points))
        return self.filtered_points

    def collect_all_points(self):
        """
        Function which arranges all the filtered points in an array. The points from all the objects are put together
        in self.filtered_points_array. No distinction is made between points of different objects by this array, as it
        was done by self.filtered_points.
        :return self.filtered_points_array: array with all the points of all the obstacles in the environment.
        """
        if self.filtered_points is None:
            self.project_points_altitude()
        object_names = list(self.filtered_points.keys())
        self.filtered_points_array = self.filtered_points[object_names[0]]
        object_names = object_names[1:]
        for name in object_names:
            self.filtered_points_array = np.vstack((self.filtered_points_array, self.filtered_points[name]))
        return self.filtered_points_array

    def fill_grid_filtered_points(self):
        """
        Fill the grid with the previously filtered points
        :return self.grid: Occupancy grid with the occupied cells equal to True
        """
        # If there are no points cloud, it has to be computed first
        if self.filtered_points_array is None:
            self.collect_all_points()

        # Obtain the number of points, the max limits and loop over all the points in the cloud
        n_points = self.filtered_points_array.shape[0]
        self.limit_x_max = self.limit_x_min + self.extent_x * self.cell_size
        self.limit_y_max = self.limit_y_min + self.extent_y * self.cell_size
        for i in range(n_points):
            x_pos = self.filtered_points_array[i, 0]
            y_pos = self.filtered_points_array[i, 1]

            # Check if the point is outside the grid bounds
            if x_pos > self.limit_x_max or x_pos < self.limit_x_min or y_pos > self.limit_y_max or \
                    y_pos < self.limit_y_min:  # Ignore points outside of grid
                pass
            else:  # Else compute point location and save it in the grid
                x_pos = int((x_pos - self.limit_x_min) / self.cell_size)
                y_pos = int((y_pos - self.limit_y_min) / self.cell_size)
                self.grid[x_pos, y_pos] = True

        return self.grid

    def fill_grid_filtered_points_objects(self):
        """
        Fill the grid with the previously filtered points. Store grid cell coordinates for each object. If there are x
        points of an object on a grid cell, they get reduce to the single cell coordinate. Very similar function to
        self.fill_grid_filtered_points
        :return self.object_grid_coord: grid coordinates for each object.
        """
        if self.filtered_points_array is None:
            self.fill_grid_filtered_points()

        # Initializes output dictionary with the objects' names as keys and the objects' occupied grid cells as values
        objects_names = self.filtered_points.keys()
        self.object_grid_coord = {key: set() for key in objects_names}

        # Iterate over all the objects
        for name in objects_names:
            points = self.filtered_points[name]
            for i in range(points.shape[0]):
                x_pos = points[i, 0]
                y_pos = points[i, 1]
                if x_pos > self.limit_x_max or x_pos < self.limit_x_min or \
                        y_pos > self.limit_y_max or y_pos < self.limit_y_min:  # Ignore points outside of grid
                    pass
                else:  # Else compute point location
                    x_pos = int((x_pos - self.limit_x_min) / self.cell_size)
                    y_pos = int((y_pos - self.limit_y_min) / self.cell_size)
                    self.object_grid_coord[name].add((x_pos, y_pos))
            self.object_grid_coord[name] = list(self.object_grid_coord[name])
        return self.object_grid_coord

    def identify_object_internal_points(self, alpha=200):
        """
        Obtains the coordinates of all the points within a polygon. Until now the objects are defined by the points
        derived from the meshes of the obstacles. This function aims at filling all the grid cells with an obstacle.
        :param alpha: consider 3 points of an obstacle and draw a triangle, then the circle circumscribed to the
        triangle. If the radius of the circle is smaller than alpha, then the points are considered to be part of a
        triangle and their connection is considered when obtaining the edges of a polygon. The default value of 200
        is used, since that most likely includes all points of the obstacle.
        :return self.object_grid_coord_all: dictionary with the coordinates of the points inside the obstacles per
        obstacle.
        """
        # Iterate over allthe objects
        objects_names = self.object_grid_coord.keys()
        self.object_grid_coord_all = self.object_grid_coord.copy()
        for name in objects_names:
            object_points = np.array(list(self.object_grid_coord[name]))  # Coordinates of each obstacle
            if object_points.shape[0] > 3 and len(set(object_points[:, 0])) > 1 and len(set(object_points[:, 1])) > 1:
                # Obtain the (grid) coordinates of the points that shape the outer edge of the polygon
                _, output_points_unordered = list(obtain_outer_edge(object_points, alpha, only_outer=True))

                # Order the points clockwise
                output_points_ordered = counter_clockwise_ordering(output_points_unordered)

                x, y = np.meshgrid(np.arange(self.grid.shape[0]),
                                   np.arange(self.grid.shape[1]))  # make a canvas with coordinates
                x, y = x.flatten(), y.flatten()
                points = np.vstack((x, y)).T

                # Make a polygon with the coordinates of the given points
                p = Path(output_points_ordered)

                # The points within the polygon are identified. A small radius is given in order to also consider
                # those points on the edges.
                grid = p.contains_points(points, radius=0.0000001)
                mask = grid.reshape(self.grid.shape[0],
                                    self.grid.shape[1])  # now you have a mask with points inside a polygon

                # Set up the grid coordinates in a 2D mesh
                points_grid = points.reshape(self.grid.shape[0], self.grid.shape[1], 2)

                # Apply mask to point grid
                internal_points = points_grid[mask]
            else:
                internal_points = object_points

            for i in range(internal_points.shape[0]):
                # All internal object points coordinates are stored
                self.object_grid_coord_all[name].append(tuple(internal_points[i, :]))

                # A grid is created only with the internal object points
                self.object_grid[internal_points[i, 0], internal_points[i, 1]] = True
            ic(name)

        return self.object_grid_coord_all

    def create_grid_full_obstacle(self):
        """
        Put together the grid of the obstacles and the grid of the points inside the obstacles.
        :return: None
        """
        self.grid = self.grid + self.object_grid

    def plot_projected_points(self):
        """
        Plot the points in 2D with matplotlib
        :return: None
        """
        import matplotlib as mpl
        mpl.rcParams['pdf.fonttype'] = 42
        mpl.rcParams['ps.fonttype'] = 42
        mpl.rcParams['font.family'] = 'Arial'
        mpl.rcParams['grid.alpha'] = 0.5
        mpl.use('TkAgg')
        font = {'size': 42,
                'family': "Arial"}
        mpl.rc('font', **font)

        # Create figure
        fig = plt.figure()
        object_names = self.filtered_points.keys()
        total_number_points = 0
        # Iterate over all the objects
        for name in object_names:
            plt.scatter(self.filtered_points[name][:, 1], self.filtered_points[name][:, 0], s=10)
            total_number_points += self.filtered_points[name][:, 0].shape[0]
        ic("Total number of points: " + str(total_number_points))
        plt.xlabel("y-coordinate")
        plt.ylabel("x-coordinate")
        plt.grid(True)
        ax = plt.gca()
        ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 1))
        ax.ticklabel_format(axis="x", style="sci", scilimits=(0, 1))
        fig.set_size_inches(19.24, 10.55)
        fig.subplots_adjust(left=0.13, top=0.95, right=0.98, bottom=0.17)
        # Uncomment the following lines if the user desires to store the file
        # location_save = ""
        # plot_name = "2D_cross"
        # fig.savefig(os.path.join(f"{location_save}", f"{plot_name}.pdf"), bbox_inches='tight')
        plt.show()

    def plot_projected_points_3D_grid(self):
        """
        Plot the points and the occupancy in an interactive 3D mesh
        :return: None
        """
        self.plotter = Plotter3D(self, interactive=True)
        self.plotter.remove_point_clouds()
        self.plotter.plot_point_cloud(self.filtered_points_array)
        self.plotter.plot_occupancygrid()

    def plot_start_goal_3D_grid(self, start, goal):
        """
        Plot the start and the goal locations in red
        :param start: location of the starting point
        :param goal: location of the target point
        :return: None
        """
        start = self.translate_point_to_UE4_coord(start, 0)
        goal = self.translate_point_to_UE4_coord(goal, 0)
        points = np.array([[start[0], start[1]], [goal[0], goal[1]]])
        self.plotter.plot_point_cloud(points, color='r', size=20.0)

    def plot_trajectory_3d_grid(self, path, color='green'):
        """
        Once the grid has been plotted in 3D, the computed path is plotted with 3D arrows.
        :param color: color of the arrows that show the path
        :param path: path that the drone must follow
        :return: None
        """
        if self.plotter is None:
            self.plot_projected_points_3D_grid()
        self.plotter.plot_trajectory(path, color=color)

    def translate_path_to_world_coord(self, path, h):
        """
        Translates a path of points in grid coordinates to world coordinates
        :param path: path of points that the drone must follow
        :param h: altitude at which the drone is going to fly
        :return world_waypoints: list of points in AirSim coordinates that the drone is going to follow
        """
        world_waypoints = []
        for point in path:
            coord = self.translate_point_to_world_coord(point, h)
            world_waypoints.append(coord)
        return world_waypoints

    def translate_path_lst_to_Vector3r(self, path):
        """
        Translates a path composed of points in list format to a list of points in Vector3r format (AirSim coordinates)
        encapsulating them in an airsim.Vector3r object
        :param path: path of coordinates
        :return: airsim.Vector3r object with the waypoints that the vehicle should follow
        """
        Vector3r_waypoints = []
        for point in path:
            point_vec = airsim.Vector3r(point[0], point[1], point[2])
            Vector3r_waypoints.append(point_vec)
        return Vector3r_waypoints

    def translate_point_to_world_coord(self, point, h):
        """
        Method that translates a single point from grid to AirSim coordinates. For that purpose, it passes first through
        the UE4 coordinate frame: grid-->UE4-->AirSim
        :param point: grid coordinates of point
        :param h: altitude at which the drone flies
        :return coord: xyz coordinates of the point in the AirSim reference frame.
        """
        x_coord = (point[0] * self.cell_size + self.limit_x_min) / self.ue4_airsim_conv
        y_coord = (point[1] * self.cell_size + self.limit_y_min) / self.ue4_airsim_conv
        coord = (x_coord, y_coord, -h / self.ue4_airsim_conv)
        return coord

    def translate_point_to_UE4_coord(self, point, h):
        """
        Translates grid point to UE4 coordinate frame
        :param point: point coordinates
        :param h: altitude at which the drone flies
        :return coord: x,y,z coordinates in the UE4 reference frame
        """
        x_coord = (point[0] * self.cell_size + self.limit_x_min)
        y_coord = (point[1] * self.cell_size + self.limit_y_min)
        coord = (x_coord, y_coord, -h)
        return coord

    def check_obstacle(self, location, distance=3):
        """
        Function which checks whether a certain point (location) can be found within an obstacle or within a certain
        distance of any obstacle.
        :param location: location to evaluate
        :param distance: distance from obstacle to be evaluated
        :return: whether the location is inside or in the distance-range of an obstacle
        """
        # Obtain all the x-y coordinates from the location within the input distance
        candidates_x = np.unique(np.minimum(np.maximum(np.arange(location[0]-distance, location[0]+distance+1), 0),
                                            self.extent_x-1))
        candidates_y = np.unique(np.minimum(np.maximum(np.arange(location[1]-distance, location[1]+distance+1), 0),
                                            self.extent_y-1))

        # Check if any of the points around of the location (including the location) are within an obstacle.
        for i in range(candidates_x.shape[0]):
            for j in range(candidates_y.shape[0]):
                if self.grid[int(candidates_x[i]), int(candidates_y[j])]:
                    # Check whether the evaluated point is still within the distance from the location
                    # The square root of 2 is added to the distance since that is the distance of the diagonal of a
                    # square. The worst case scenario.
                    if compute_distance_points((candidates_x[i], candidates_y[j]), location) <= (distance+sqrt(2)):
                        return True
        return False

    def run(self, h=1100, delta_h=500, filename_vertices='object_points',
            update_vertices_flag=False, plot_2D=False, plot3D=False):
        """
        Function which computes the Occupancy map. First, the obstacles are extracted, they are projected to the x-y
        plane and the environment dimensions are defined. Then, the grid is filled with the obstacle meshes and the
        inner obstacle points are identified. Finally, both meshes are put together
        :param h: altitude at which the point cloud is sliced
        :param delta_h: altitude range of the slice
        :param filename_vertices: location where the point cloud is stored
        :param update_vertices_flag: whether the saved points are updated
        :param plot_2D: whether the 2D plots are generated
        :param plot3D: whether the 3D plots are generated
        :return: None
        """
        self.extract_obstacle_vertices(filename=filename_vertices, update_points=update_vertices_flag)
        self.project_points_altitude(h, delta_h)
        self.set_env_dims()
        _ = self.fill_grid_filtered_points()
        if plot_2D:
            self.plot_projected_points()

        _ = self.fill_grid_filtered_points_objects()
        self.identify_object_internal_points()
        self.create_grid_full_obstacle()
        if plot3D:
            self.plot_projected_points_3D_grid()


if __name__ == "__main__":
    # Simple implementation which shows the functionality of the Occupancy map class
    # User input
    args = load_user_input()
    altitude = args.altitude_m * args.ue4_airsim_conversion_units
    cell_size = args.cell_size_m * args.ue4_airsim_conversion_units
    altitude_range = args.altitude_range_m * args.ue4_airsim_conversion_units

    # Extract occupancy grid
    env_map = OccupancyMap(cell_size=args.cell_size, ue4_airsim_conv=args.ue4_airsim_conversion_units)
    env_map.run(altitude, altitude_range, args.saved_vertices_filename, args.update_saved_vertices, args.plot_2D,
                args.plot_3D)
