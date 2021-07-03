from OccupancyMap import OccupancyMap
from PythonRobotics.PathPlanning.WavefrontCPP.wavefront_coverage_path_planner import wavefront
from PythonRobotics.PathPlanning.VoronoiRoadMap.voronoi_road_map import VoronoiRoadMapPlanner
from PythonRobotics.PathPlanning.InformedRRTStar.informed_rrt_star import InformedRRTStar
from PythonRobotics.PathPlanning.AStar.a_star import AStarPlanner
from PythonRobotics.PathPlanning.BSplinePath.bspline_path import approximate_b_spline_path, interpolate_b_spline_path
from PythonRobotics.PathPlanning.ProbabilisticRoadMap.probabilistic_road_map import prm_planning
from user_input import load_user_input
from math import isclose

import numpy as np
import matplotlib.pyplot as plt


class GridNavigation:
    """
    Class which carries out the navigation of the drone flight given a computed grid occupancy map
    """

    def __init__(self, om: OccupancyMap, plotter_2D=True, plotter_3D=True):
        self.om = om
        self.grid = om.grid
        self.plotter_2D = plotter_2D
        self.plotter_3D = plotter_3D

    def navigate_wavefront(self, start, goal):
        """
        Wavefront navigation as explained in https://pythonrobotics.readthedocs.io/en/latest/
        :param start: start location
        :param goal: target location
        :return path: path of points in grid coordinates that the drone must follow
        """
        grid_wavefront = self.grid.astype('float')
        for row in range(grid_wavefront.shape[0]):
            number = row
            for column in range(grid_wavefront.shape[1]):
                if isclose(grid_wavefront[row, column], 0, abs_tol=1e-6):
                    grid_wavefront[row, column] = number
                    number += 1
                else:
                    grid_wavefront[row, column] = float('inf')
        path = wavefront(grid_wavefront, start, goal)

        if self.plotter_3D:
            self.om.plot_trajectory_3d_grid(path)

        print('Wavefront path has been computed')
        return path

    def navigate_Voronoid(self, start, goal, robot_size=1):
        """
        Voronoid navigation as explained in https://pythonrobotics.readthedocs.io/en/latest/
        :param start: start location
        :param goal: target location
        :param robot_size: size of the robot in order to maintain a minimum distance from the obstacles and avoid
        collisions
        :return path: path of points in grid coordinates that the drone must follow
        """
        # Start and goal coordinates
        sx = start[0]
        sy = start[1]
        gx = goal[0]
        gy = goal[1]

        # Obstacle coordinates
        ox = np.where(self.grid == True)[0].tolist()
        oy = np.where(self.grid == True)[1].tolist()

        rx, ry = VoronoiRoadMapPlanner(only_integers=True, show_animation=self.plotter_2D).planning(sx, sy, gx, gy, ox,
                                                                                                    oy, robot_size)
        path = [(i, j) for i, j in zip(rx, ry)]

        if self.plotter_2D:
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "^r")
            plt.plot(gx, gy, "^c")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(rx, ry, "-r")
            plt.pause(0.1)
            plt.show()

        if self.plotter_3D:
            self.om.plot_trajectory_3d_grid(path)

        print('Voronoid path has been computed')
        return path

    def navigation_RRT_star(self, start, goal):
        """
        RRT_star navigation as explained in https://pythonrobotics.readthedocs.io/en/latest/
        :param start: start location
        :param goal: target location
        :return path: path of points in grid coordinates that the drone must follow
        """
        print("Start informed rrt star planning")

        # Obstacle coordinates
        ox = np.where(self.grid == True)[0].tolist()
        oy = np.where(self.grid == True)[1].tolist()

        # Create obstacle grid which includes the size of the obstacle as a 3rd tuple component
        obstacleList = [(i, j, 1) for i, j in zip(ox, oy)]

        # Obtain max distance as the maximum distance that will be inspected by the navigation algorithm
        max_distance = np.max(self.grid.shape)

        # Set params
        rrt = InformedRRTStar(start=list(goal), goal=list(start),
                              randArea=[0, max_distance], obstacleList=obstacleList, only_integer=True,
                              expandDis=5, maxIter=30)
        path = rrt.informed_rrt_star_search(animation=self.plotter_2D)
        print("Done!!")

        # Plot path
        if self.plotter_2D:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)
            plt.show()

        if self.plotter_3D:
            self.om.plot_trajectory_3d_grid(path)

        print('RRT_star path has been computed')
        return path

    def navigation_A_star(self, start, goal, robot_radius=3):
        """
        A*  navigation as explained in https://pythonrobotics.readthedocs.io/en/latest/
        :param start: start location
        :param goal: target location
        :param robot_size: size of the robot in order to maintain a minimum distance from the obstacles and avoid
        collisions
        :return path: path of points in grid coordinates that the drone must follow
        """
        # Coordinates of the start and goal locations
        sx = start[0]
        sy = start[1]
        gx = goal[0]
        gy = goal[1]
        grid_size = 1

        # Coordinates of the obstacles
        ox = np.where(self.grid == True)[0].tolist()
        oy = np.where(self.grid == True)[1].tolist()

        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy, self.plotter_2D)
        path = [(i, j) for i, j in zip(rx, ry)]

        # Reverse the path because the A* algorithm returns the path starting from the goal.
        path.reverse()

        if self.plotter_2D:
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(rx, ry, "-r")
            plt.pause(0.001)
            plt.show()

        if self.plotter_3D:
            self.om.plot_trajectory_3d_grid(path)

        print('A_star path has been computed')
        return path

    def smooth_B_spline(self, path, reduction=0.5):
        """
        B spline navigation path smoother as explained in https://pythonrobotics.readthedocs.io/en/latest/
        :param path: original path from one of the other navigation algorithms
        :param reduction: percentage of path waypoints that should be maintained [0,1]
        :return path_a: smoothed path of points in grid coordinates that the drone must follow
                collision: whether the generated path collides with any of the created obstacles
        """
        # Obtain the x and y coordinates of the points along the path
        way_point_x = [i[0] for i in path]
        way_point_y = [i[1] for i in path]
        n_course_point = max(int(len(path) * reduction), 2)  # sampling number

        rax, ray = approximate_b_spline_path(way_point_x, way_point_y,
                                             n_course_point, degree=2)
        path_a = [(round(i), round(j)) for i, j in zip(rax, ray)]

        # It is checked whether the proposed path collides with the obstacles
        collision = self.check_collision(path_a)

        # rix, riy = interpolate_b_spline_path(way_point_x, way_point_y,
        #                                      n_course_point, degree=2)
        # path_i = [(i, j) for i, j in zip(rix, riy)]

        # show results
        if self.plotter_2D and not collision:
            plt.plot(way_point_x, way_point_y, '-og', label="way points")
            plt.plot(rax, ray, '-r', label="Approximated B-Spline path")
            # plt.plot(rix, riy, '-b', label="Interpolated B-Spline path")
            plt.grid(True)
            plt.legend()
            plt.axis("equal")

            # self.om.plot_trajectory_3d_grid(path_i)
            plt.show()

        if self.plotter_3D and not collision:
            self.om.plot_trajectory_3d_grid(path_a)

        return path_a, collision

    def check_collision(self, path, distanced_obs=True):
        """
        Check whether the proposed path collides with an obstacle. For that purpose, the trajectory is sliced 1000 times
        and it is checked whether the grid cell of any of the points along the sliced path falls on an occupied cell.
        If there is an obstacle, then return True
        :param path: proposed path that the drone must follow
        :param distanced_obs: whether a minimum distance of 1 cell should be kept from the obstacles
        :return: whether the path collides with an obstacle
        """
        # For each connection between 2 points along the path
        for i in range(len(path) - 1):
            # Retrieve the 2 points
            local_start = path[i][:2]
            local_goal = path[i + 1][:2]
            # Check whether the trajectory moves only along the y-direction
            if local_goal[0] != local_start[0]:
                # Obtain the slope of the line connecting both points
                m = (local_goal[1] - local_start[1]) / (local_goal[0] - local_start[0])
                line_equation = lambda x: m * (x - local_start[0]) + local_start[1]

                if distanced_obs:
                    line_equation_parallel_1 = lambda x: m * (x - local_start[0] - 1) + local_start[1]
                    line_equation_parallel_2 = lambda x: m * (x - local_start[0] + 1) + local_start[1]

                    # Obtain linspace of the x axis
                    x0 = np.linspace(local_start[0], local_goal[0], 1000)
                    x1 = np.linspace(local_start[0] + 1, local_goal[0] + 1, 1000)
                    x2 = np.linspace(local_start[0] - 1, local_goal[0] - 1, 1000)

                    # Obtain the corresponding coordinates in the y-direction
                    y = np.array(list(map(line_equation, x0)) + list(map(line_equation_parallel_1, x1)) + list(
                        map(line_equation_parallel_2, x2)))
                    x = np.vstack((x0, x1, x2)).flatten()
                else:
                    x = np.linspace(local_start[0], local_goal[0], 1000)
                    y = np.array(list(map(line_equation, x)))
            else:  # Only slice the y-axis
                if distanced_obs:
                    x0 = np.ones(1000) * local_start[0]
                    x1 = np.ones(1000) * local_start[0] + 1
                    x2 = np.ones(1000) * local_start[0] - 1
                    x = np.vstack((x0, x1, x2)).flatten()
                    y0 = np.linspace(local_start[1], local_goal[1], 1000)
                    y = np.vstack((y0, y0, y0)).flatten()
                else:
                    x = np.ones(1000) * local_start[0]
                    y = np.linspace(local_start[1], local_goal[1], 1000)

            x_int = np.round(x).astype(int)
            y_int = np.round(y).astype(int)
            coord = np.vstack((x_int, y_int)).T
            grids_cells = np.unique(coord, axis=0)

            # Check whether the cells along the trajectory are occupied by an obstacle
            for cell in grids_cells:
                if self.om.extent_x > cell[0] >= 0 and self.om.extent_y > cell[1] >= 0:
                    if self.grid[cell[0], cell[1]]:
                        return True
        return False

    def navigation_PRM(self, start, goal, robot_size=3):
        """
        Probabilistic Road Map navigation as explained in https://pythonrobotics.readthedocs.io/en/latest/
        :param start: start location
        :param goal: target location
        :param robot_size: size of the robot in order to maintain a minimum distance from the obstacles and avoid
        collisions
        :return path: path of points in grid coordinates that the drone must follow
        """
        # Coordinates of the start and goal positions
        sx = start[0]
        sy = start[1]
        gx = goal[0]
        gy = goal[1]

        # Coordinates of the obstacle locations
        ox = np.where(self.grid == True)[0].tolist()
        oy = np.where(self.grid == True)[1].tolist()

        rx, ry = prm_planning(sx, sy, gx, gy, ox, oy, robot_size, self.plotter_2D)
        path = [(i, j) for i, j in zip(rx, ry)]

        # The path is reversed because the PRM algorithm returns the path starting from the goal
        path.reverse()

        if self.plotter_2D:
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "^r")
            plt.plot(gx, gy, "^c")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(rx, ry, "-r")
            plt.pause(0.001)
            plt.show()

        if self.plotter_3D:
            self.om.plot_trajectory_3d_grid(path)

        print('PRM path has been computed')

        return path


if __name__ == "__main__":
    # User input
    args = load_user_input()
    altitude = args.altitude_m * args.ue4_airsim_conversion_units
    cell_size = args.cell_size_m * args.ue4_airsim_conversion_units
    altitude_range = args.altitude_range_m * args.ue4_airsim_conversion_units

    # Extract occupancy grid
    env_map = OccupancyMap(cell_size=cell_size)
    env_map.run(altitude, altitude_range, args.saved_vertices_filename, args.update_saved_vertices,
                args.plot2D, args.plot3D)
    nav = GridNavigation(env_map, args.plot2D, args.plot3D)
    # nav.navigate_wavefront(args.start, args.goal)
    # nav.navigate_Voronoid(args.start, args.goal, args.robot_radius)
    # nav.navigation_RRT_star(args.start, args.goal)
    path = nav.navigation_A_start(args.start, args.goal, args.robot_radius)
    path, collision = nav.smooth_B_spline(path, reduction=0.2)
    # path = nav.navigation_PRM(args.start, args.goal, args.robot_radius)
    # path = nav.smooth_B_spline(path, reduction=0.5)
