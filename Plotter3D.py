import numpy as np
import pyvista as pv


COLORS = ['b', 'g', 'r', 'w']


class Plotter3D(pv.Plotter):
    def __init__(self, occupancy_grid, interactive=False):
        self.occupancy_grid = occupancy_grid
        self.prev_point_clouds = []
        self.current_grid_actor = None

        super(Plotter3D, self).__init__()
        self.background_color = '#FFFFFF'  # '#f0f0f0'

        # Arrow colors: X -> Red, Y -> Green, Z -> Blue (RGB).
        self.add_axes_at_origin(labels_off=True)
        if interactive:
            # camera_pos = (-1,-1,0)
            # up = (0,0,1)
            # focus_point = (0,0,0)
            camera_pos = (0, 0, 90)
            up = (1, 0, 0)
            focus_point = (0, 0, 0)
            self.show(cpos=[camera_pos, focus_point, up],
                      interactive_update=True, auto_close=False)

    def remove_point_clouds(self):
        if len(self.prev_point_clouds) > 0:
            for pc in self.prev_point_clouds:
                if pc is not None:
                    self.remove_actor(pc)
        self.update()

    def plot_point_cloud(self, points, img=None, color='b', size=3.0):
        # Add third dimension if non-existent
        if len(points.shape) != 3:
            z_points = np.ones((points.shape[0], 1)) * -2
            points = np.hstack((points, z_points))

        # Remove NaN points.
        nonnan_mask = ~np.any(np.isnan(points), axis=1)
        points = points[nonnan_mask]

        actor = None
        pc = pv.PolyData(points)
        if img is None:
            actor = self.add_mesh(
                        pc, color=color,
                        style='points', point_size=size)
        else:
            img = img[nonnan_mask]
            actor = self.add_mesh(
                pc, scalars=img,
                style='points', point_size=size, rgb=True, smooth_shading=True)

            pass
        self.prev_point_clouds.append(actor)
        # self.update()

    def plot_occupancygrid(self):
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

        self.current_grid_actor = self.add_mesh(grid_pv, texture=tex, show_edges=True)
        self.update()

    def plot_pose(self, position, orientation, color='k', opacity=0.5):
        arrow = pv.Arrow(start=position,
                         direction=orientation,
                         scale='auto')
        return self.add_mesh(arrow, color=color, opacity=opacity)

    def plot_trajectory(self, path, color='green', step=1, height=-1):
        cell_size = self.occupancy_grid.cell_size
        x_min = self.occupancy_grid.limit_x_min
        y_min = self.occupancy_grid.limit_y_min
        translation = [(i[0]*cell_size+x_min, i[1]*cell_size+y_min, height) for i in path]
        orientations = self.compute_orientation_trajectory(translation)

        for idx, (position, orientation) in enumerate(zip(translation, orientations)):
            if idx % step == 0:
                self.plot_pose(position, orientation, color)
        self.update()

    def compute_orientation_trajectory(self, translation):
        orientations = []
        for i in range(len(translation)-1):
            x = translation[i+1][0] - translation[i][0]
            y = translation[i + 1][1] - translation[i][1]
            z = 0
            orientations.append((x, y, z))
        return orientations


