import argparse


# %% Input parameters
def load_user_input():
    """
        Function that stores the user input
    Returns:
        parser.parse_args(): it passes the parser state
    :return:
    """
    parser = argparse.ArgumentParser()

    parser.add_argument('--altitude_m', type=int, default=-11, help='Drone flight altitude: m')
    parser.add_argument('--altitude_range_m', type=int, default=3,
                        help='Altitude range to slice the cloud of points: m')
    parser.add_argument('--cell_size_m', type=int, default=3, help='Grid cell size: m')
    parser.add_argument('--ue4_airsim_conversion_units', type=int, default=100,
                        help='Conversion factor from Unreal Engine 4 to Airsim units (m)')
    parser.add_argument('--min_flight_distance_m', type=int, default=30,
                        help='Minimum distance that a drone must fly in order to be considered a flight')
    parser.add_argument('--saved_vertices_filename', type=str, default='object_points',
                        help='Directory where to save cloud points')
    parser.add_argument('--update_saved_vertices', type=bool, default=False,
                        help='Whether the saved cloud points should be saved')
    parser.add_argument('--plot2D', type=bool, default=False, help='Whether the 2D plots should be shown.')
    parser.add_argument('--plot3D', type=bool, default=True, help='Whether the 3D plots should be shown.')
    parser.add_argument('--start', default=(7, 10), help='Starting flight location (tuple). If None, random point.')
    parser.add_argument('--goal', default=(30, 43), help='Target flight location (tuple). If None, random point.')
    parser.add_argument('--robot_radius', type=int, default=9,
                        help='Size of the robot in order to maintain a minimum distance'
                             'to the obstacles for the A_star, Voronoid and PRM algorithms: m')
    parser.add_argument('--smooth', type=bool, default=True, help='Whether the path is smoothed with B-splines.')
    parser.add_argument('--sensors_lst', type=list, default=['barometer', 'gps', 'magnetometer', 'imu'],
                        help='List of sensors to use')
    parser.add_argument('--cameras_info', type=dict, default={'front': {"camera_name": "0", "image_type": 0}},
                        help='Dictionary cotaining the camera information: '
                             'alias, camera_name, image_type, pixels_as_float and compress')
    parser.add_argument('--sample_rate', type=int, default=10,
                        help='Sampling rate for the sensors.')
    parser.add_argument('--number_runs', type=int, default=10,
                        help='Number of runs to be performed')
    parser.add_argument('--navigation_type', type=str, default="A_star",
                        help='Method employed for navigation: A_star, wavefront, Voronoid, RRT_star and PRM')
    parser.add_argument('--flight_altitudes', default=[2, 11],
                        help="Range of altitudes at which the drone could be spawned")

    return parser.parse_args()
