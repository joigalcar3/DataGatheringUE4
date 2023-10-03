#!/usr/bin/env python
"""
Provides the tools to represent the occupancy map and the vehicle trajectories in an interactive 3D environment.
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
import numpy as np
import pyvista as pv


COLORS = ['b', 'g', 'r', 'w']


class Plotter3D(pv.Plotter):
    """
    Class which enables the 3D visualization and manipulation of the occupancy map used for computing the navigation of
    the vehicle in the UE4 environment without colliding against obstacles.
    """
    def __init__(self, occupancy_grid, interactive=False):
        """
        Initializes the Plotter3D object
        :param occupancy_grid: the array that represents the occupancy grid of filled and empty cells
        :param interactive: whether the plot should or not be interactive
        """
        self.occupancy_grid = occupancy_grid
        self.prev_point_clouds = []
        self.current_grid_actor = None

        super(Plotter3D, self).__init__()
        self.background_color = '#FFFFFF'

        # Add axes at the origin and enable the interactive mode
        self.add_axes_at_origin(labels_off=True)
        if interactive:
            camera_pos = (0, 0, 90)
            up = (1, 0, 0)
            focus_point = (0, 0, 0)
            self.show(cpos=[camera_pos, focus_point, up],
                      interactive_update=True, auto_close=False)

    def remove_point_clouds(self):
        """
        Method which cleans the 3D plotter canvas for a future plot
        :return: None
        """
        if len(self.prev_point_clouds) > 0:
            for pc in self.prev_point_clouds:
                if pc is not None:
                    self.remove_actor(pc)
        self.update()

    def plot_point_cloud(self, points, color='b', size=3.0):
        """
        Method which plots a point cloud on the 3D space
        :param points: the coordinates of the points
        :param color: the color with which the points should be plotted
        :param size: the size of the points to be plotted
        :return: None
        """
        # Add third dimension if non-existent
        if len(points.shape) != 3:
            z_points = np.ones((points.shape[0], 1)) * -2
            points = np.hstack((points, z_points))

        # Remove NaN points.
        nonnan_mask = ~np.any(np.isnan(points), axis=1)
        points = points[nonnan_mask]

        # Create point cloud
        pc = pv.PolyData(points)
        actor = self.add_mesh(
                    pc, color=color,
                    style='points', point_size=size)
        self.prev_point_clouds.append(actor)

    def plot_occupancygrid(self):
        """
        Plot the 3D occupancy map to the user's screen
        :return: None
        """
        # Remove any pre-existing occupancy map mesh in the 3D plot
        if self.current_grid_actor is not None:
            self.remove_actor(self.current_grid_actor)
        grid_array = self.occupancy_grid.grid
        cell_size = self.occupancy_grid.cell_size

        # Create the spatial reference
        dims = np.array([1, 1, 1])
        dims[:2] = np.array(grid_array.shape)+1
        grid_pv = pv.UniformGrid(dims)

        grid_pv.spacing = (cell_size, cell_size, 1)  # These are the cell sizes along each axis

        # The bottom left corner of the data set
        grid_pv.origin = (self.occupancy_grid.limit_x_min,
                          self.occupancy_grid.limit_y_min,
                          0)

        # Rotate and flip to align texture.
        grid_array = grid_array.T
        grid_array = grid_array[::-1, :]
        grid_array = 1-grid_array

        tex = np.stack([grid_array*255]*3, axis=2).astype(np.uint8)
        tex = pv.numpy_to_texture(tex)
        grid_pv.texture_map_to_plane(inplace=True)

        # Add mesh to the 3D plot
        self.current_grid_actor = self.add_mesh(grid_pv, texture=tex, show_edges=True)
        self.update()

    def plot_pose(self, position, orientation, color='k', opacity=0.5):
        """
        Method to plot an arrow in the 3D plot environment
        :param position: root location of the arrow
        :param orientation: tip location of the arrow using the root as the center of coordinates
        :param color: color of the arrow
        :param opacity: opacity of the arrow
        :return: an arrow actor
        """
        arrow = pv.Arrow(start=position,
                         direction=orientation,
                         scale='auto')
        return self.add_mesh(arrow, color=color, opacity=opacity)

    def plot_trajectory(self, path, color='green', step=1, height=-1):
        """
        Method that plots the trajectory of the vehicle with arrows
        :param path: the coordinates of all the path points that the vehicle should follow
        :param color: the color of the trajectory arrows
        :param step: whether the arrows should be plotted in intermittent intervals. For example, if "step"=2 only the
        even arrows are plotted to represent the trajectory.
        :param height: the distance from the 2D plane that is the occupancy map
        :return:
        """
        cell_size = self.occupancy_grid.cell_size
        x_min = self.occupancy_grid.limit_x_min
        y_min = self.occupancy_grid.limit_y_min

        # Obtain the start and end location of the arrows
        translation = [(i[0]*cell_size+x_min, i[1]*cell_size+y_min, height) for i in path]
        orientations = self.compute_orientation_trajectory(translation)

        # Plot the arrows
        for idx, (position, orientation) in enumerate(zip(translation, orientations)):
            if idx % step == 0:
                self.plot_pose(position, orientation, color)
        self.update()

    def compute_orientation_trajectory(self, translation):
        """
        Method that translates the coordinates of the tip of the arrows from the occupancy coordinate frame to a
        coordinate frame centered at the root of the arrow.
        :param translation: array containing the waypoints of the trajectory that the vehicle should follow and that
        should be represented in the form of arrows
        :return: the transformed coordinates
        """
        orientations = []
        for i in range(len(translation)-1):
            x = translation[i+1][0] - translation[i][0]
            y = translation[i + 1][1] - translation[i][1]
            z = 0
            orientations.append((x, y, z))
        return orientations


