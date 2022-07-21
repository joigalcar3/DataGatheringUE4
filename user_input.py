import argparse
from user_input_file_loc import load_user_input_file_loc


# %% Input parameters
def load_user_input():
    """
        Function that stores the user input
    Returns:
        parser.parse_args(): it passes the parser state
    :return:
    """
    parser = argparse.ArgumentParser()

    # parser.add_argument('--altitude_m', type=int, default=0, help='Drone flight altitude: m')
    # parser.add_argument('--altitude_range_m', type=int, default=3,
    #                     help='Altitude range to slice the cloud of points: m')
    # parser.add_argument('--cell_size_m', type=float, default=6, help='Grid cell size: m')
    # parser.add_argument('--ue4_airsim_conversion_units', type=int, default=100,
    #                     help='Conversion factor from Unreal Engine 4 to Airsim units (m)')
    # parser.add_argument('--min_flight_distance_m', type=int, default=30,
    #                     help='Minimum distance that a drone must fly in order to be considered a flight')
    # parser.add_argument('--max_flight_distance_m', type=int, default=200,
    #                     help='Maximum distance that a drone must fly in order to be considered a flight')
    # parser.add_argument('--saved_vertices_filename', type=str, default='Temple_object_points',
    #                     help='Directory where to save cloud points')
    # parser.add_argument('--update_saved_vertices', type=bool, default=False,
    #                     help='Whether the saved cloud points should be saved')
    # parser.add_argument('--plot2D', type=bool, default=False, help='Whether the 2D plots should be shown.')
    # parser.add_argument('--plot3D', type=bool, default=True, help='Whether the 3D plots should be shown.')
    # parser.add_argument('--start', default=None, help='Starting flight location (tuple). If None, random point.')
    # parser.add_argument('--goal', default=None, help='Target flight location (tuple). If None, random point.')
    # parser.add_argument('--robot_radius', type=int, default=3,
    #                     help='Size of the robot in order to maintain a minimum distance'
    #                          'to the obstacles for the A_star, Voronoid and PRM algorithms: m')
    # parser.add_argument('--smooth', type=bool, default=True, help='Whether the path is smoothed with B-splines.')
    # parser.add_argument('--sensors_lst', type=list, default=['barometer', 'gps', 'magnetometer', 'imu'],
    #                     help='List of sensors to use')
    # parser.add_argument('--cameras_info', type=dict, default={'front': {"camera_name": "0", "image_type": 0}},
    #                     help='Dictionary cotaining the camera information: '
    #                          'alias, camera_name, image_type, pixels_as_float and compress')
    # parser.add_argument('--sample_rates', default={'camera': 60, 'imu': 1000, 'magnetometer': 50,
    #                                                'gps': 50, 'barometer': 50},
    #                     help='Sampling rate for the sensors. Except the camera, the default sampling rates are'
    #                          ' from the original c++ code, in the simpleParams files for each sensor.')
    # parser.add_argument('--number_runs', type=int, default=50,
    #                     help='Number of runs to be performed')
    # parser.add_argument('--navigation_type', type=str, default="A_star",
    #                     help='Method employed for navigation: A_star, wavefront, Voronoid, RRT_star and PRM')
    # parser.add_argument('--flight_altitudes', default=[3, 11],
    #                     help="Range of altitudes at which the drone could be spawned")
    # parser.add_argument('--failure_types', default=["prop_fly_off_dis_abr", "actuator_saturation_dis_abr"],
    #                     help="Failures types considered during the flight. Options listed in the Failure Factory. It"
    #                          "needs to be followed by dis or con, which tells the factory the number of options"
    #                          "considered for failure and abr or lin, which tells the factory the time component of the "
    #                          "failure.")
    # parser.add_argument('--activate_take_off', type=bool, default=False,
    #                     help="Whether the take-off should be activated.")

    # Arguments related to the altitude selection
    parser.add_argument('--altitude_m', type=int, default=-30, help='Drone flight altitude: m')
    parser.add_argument('--altitude_range_m', type=int, default=5,
                        help='Altitude range to slice the cloud of points: m')
    parser.add_argument('--flight_altitudes', default=[6, 50],
                        help="Range of altitudes at which the drone could be spawned")
    parser.add_argument('--constant_altitude_iterations', type=int, default=50,
                        help="Number of runs that the altitude is maintained constant in order to reduce the"
                             "computation load by not extracting the occupancy map. If it is 1, then the map is"
                             "extracted in every run.")

    # Arguments related to the occupancy map generation
    parser.add_argument('--cell_size_m', type=float, default=5, help='Grid cell size: m')
    parser.add_argument('--ue4_airsim_conversion_units', type=int, default=100,
                        help='Conversion factor from Unreal Engine 4 to Airsim units (m)')
    parser.add_argument('--update_saved_vertices', type=bool, default=False,
                        help='Whether the saved cloud points should be saved')

    # Arguments related to the drone navigation
    parser.add_argument('--min_flight_distance_m', type=int, default=20,   # 30
                        help='Minimum distance that a drone must fly in order to be considered a flight')
    parser.add_argument('--max_flight_distance_m', type=int, default=50,   # 200
                        help='Maximum distance that a drone must fly in order to be considered a flight')
    parser.add_argument('--start', default=None, help='Starting flight location (tuple). If None, random point.') #(25, 37)  (27, 68) (25, 37) (38, 50) (25, 42) (36, 32) (34, 42) (16, 30)  (10, 42), (24, 42)
    parser.add_argument('--goal', default=None,
                        help='Target flight location (tuple). If None, random point.')  # (50, 50)  (11, 7) (38, 50) (25, 37) (36, 32) (25, 42) (10, 42) (16, 54)  (34, 42), (14, 42)
    parser.add_argument('--robot_radius', type=int, default=9,
                        help='Size of the robot in order to maintain a minimum distance'
                             'to the obstacles for the A_star, Voronoid and PRM algorithms: m')
    parser.add_argument('--smooth', type=bool, default=True, help='Whether the path is smoothed with B-splines.')

    # Arguments related to the sensor data collection
    parser.add_argument('--sensors_lst', type=list, default=['imu', 'barometer', 'gps', 'magnetometer'],
                        help='List of sensors to use')
    parser.add_argument('--cameras_info', type=dict, default={'front': {"camera_name": "0", "image_type": 0}},
                        help='Dictionary containing the camera information: '
                             'alias, camera_name, image_type, pixels_as_float and compress')
    parser.add_argument('--sample_rates', default={'camera': 30, 'imu': 512, 'magnetometer': 30,
                                                   'gps': 30, 'barometer': 30},
                        help='Sampling rate for the sensors. Except the camera, the default sampling rates are'
                             ' from the original c++ code, in the simpleParams files for each sensor.')

    # Information regarding the navigation of the drone  sensors_remote_storage_location
    parser.add_argument('--navigation_type', type=str, default="A_star",
                        help='Method employed for navigation: A_star, wavefront, Voronoid, RRT_star and PRM')

    # Arguments related to the failure factory
    parser.add_argument('--failure_types', default=['prop_damage_advanced_single_blade_dis_abr'],  #'prop_fly_off_dis_abr', 'actuator_saturation_dis_abr', 'prop_damage_advanced_single_blade_dis_abr', 'actuator_locked_dis_abr'
                        help="Failures types considered during the flight. Options listed in the Failure Factory. It"
                             "needs to be followed by dis or con (or mix), which tells the factory the number of "
                             "options considered for failure and abr or lin, which tells the factory the time component"
                             "of the failure.")

    # Arguments related to the drone flight
    parser.add_argument('--activate_take_off', type=bool, default=False,
                        help="Whether the take-off should be activated.")

    # Arguments related to the data gathering
    parser.add_argument('--number_runs', type=int, default=1000,
                        help='Number of runs to be performed')

    # Arguments for debugging purposes
    parser.add_argument('--plot2D', type=bool, default=False, help='Whether the 2D plots should be shown.')
    parser.add_argument('--plot3D', type=bool, default=False, help='Whether the 3D plots should be shown.')

    # Arguments for controller tuning
    parser.add_argument('--controller_tuning_switch', type=bool, default=False,
                        help='Whether the plotting for PID controller tuning is activated.')
    parser.add_argument('--PSO_tuning_switch', type=bool, default=False,
                        help='Whether the plotting for PID controller tuning is activated.')
    parser.add_argument('--save_scope_images', type=bool, default=False,
                        help='Whether the scope plots should be stored.')
    parser.add_argument('--data_gather_types', default=['position', 'posref', 'yawref', 'orientation', 'velref', 'vel',
                                                        'pqrref', 'pqr', 'omegas', 'thrustref', 'accref',
                                                        'positionintegrator', 'thrustpi', 'yawtransferfcn',
                                                        'poserror', 'poserrordot', 'damagedmassforces',
                                                        'damagedmassmoments', 'damagedaeroforces', 'damagedaeromoments',
                                                        'timeinfo'],
                        help='Data that has to be gathered for the controller tuning')
    parser.add_argument('--plotting_controller_signals', default=[['posref.pos_ref_x', 'position.positions_x'],
                                                                  ['posref.pos_ref_y', 'position.positions_y'],
                                                                  ['posref.pos_ref_z', 'position.positions_z'],
                                                                  ['yawref.yaw_ref_corrected',
                                                                   'orientation.orientation_z'],
                                                                  ['orientation.orientation_x',
                                                                   'orientation.orientation_y',
                                                                   'orientation.orientation_z'],
                                                                  ['yawref.yaw_ref', 'yawref.yaw_ref_corrected'],
                                                                  ['velref.vel_ref_x', 'vel.vel_x'],
                                                                  ['velref.vel_ref_y', 'vel.vel_y'],
                                                                  ['velref.vel_ref_z', 'vel.vel_z'],
                                                                  ['pqrref.pqr_ref_x', 'pqr.pqr_x'],
                                                                  ['pqrref.pqr_ref_y', 'pqr.pqr_y'],
                                                                  ['pqrref.pqr_ref_z', 'pqr.pqr_z'],
                                                                  ['omegas.front_left', 'omegas.front_right',
                                                                   'omegas.back_right', 'omegas.back_left'],
                                                                  ['thrustref.current_thrust_ref_fb',
                                                                   'thrustref.current_thrust_ref_ff'],
                                                                  ['accref.acc_ref_x', 'accref.acc_ref_y',
                                                                   'accref.acc_ref_z'],
                                                                  ['positionintegrator.position_integrator_x',
                                                                   'positionintegrator.position_integrator_y',
                                                                   'positionintegrator.position_integrator_z'],
                                                                  ['thrustpi.thrust_P', 'thrustpi.thrust_I'],
                                                                  ['yawtransferfcn.yaw_transfer_fcn_3',
                                                                   'yawtransferfcn.yaw_transfer_fcn_1',
                                                                   'yawtransferfcn.yaw_transfer_fcn_1_1'],
                                                                  ['poserror.pos_error_x',
                                                                   'poserror.pos_error_y',
                                                                   'poserror.pos_error_z'],
                                                                  ['poserrordot.pos_error_dot_x',
                                                                   'poserrordot.pos_error_dot_y',
                                                                   'poserrordot.pos_error_dot_z'],
                                                                  ['damagedmassforces.damaged_mass_forces_x',
                                                                   'damagedmassforces.damaged_mass_forces_y',
                                                                   'damagedmassforces.damaged_mass_forces_z'],
                                                                  ['damagedmassmoments.damaged_mass_moments_x',
                                                                   'damagedmassmoments.damaged_mass_moments_y',
                                                                   'damagedmassmoments.damaged_mass_moments_z'],
                                                                  ['damagedaeroforces.damaged_aero_forces_x',
                                                                   'damagedaeroforces.damaged_aero_forces_y',
                                                                   'damagedaeroforces.damaged_aero_forces_z'],
                                                                  ['damagedaeromoments.damaged_aero_moments_x',
                                                                   'damagedaeromoments.damaged_aero_moments_y',
                                                                   'damagedaeromoments.damaged_aero_moments_z'],
                                                                  ['timeinfo.time'],
                                                                  ['timeinfo.sampling_frequency']],
                        help='Which signals have to be plotted for the tuning of the controller')
    parser.add_argument('--plotting_controller_signals_aeo', default=[[['position.positions_y', 'position.positions_x'],
                                                                       ['posref.pos_ref_y', 'posref.pos_ref_x']],
                                                                      [['position.positions_y', 'position.positions_x',
                                                                        'position.positions_z'],
                                                                       ['posref.pos_ref_y', 'posref.pos_ref_x',
                                                                        'posref.pos_ref_z']]],
                        help='Signals to be plotted against each other instead of along their timestamp/index')

    parser = load_user_input_file_loc(parser)

    return parser.parse_args()

# import airsim
# import time

# client = airsim.MultirotorClient()
# before = client.getMultirotorState().timestamp
# elapsed_lst = []
# for i in range(1000):
#     now = client.getMultirotorState().timestamp
#     elapsed_time = now-before
#     elapsed_lst.append(elapsed_time)
#     # print(elapsed_time)
#     time.sleep(1)
#     before = now
# print('Elapsed_time_mean: ', sum(elapsed_lst)/len(elapsed_lst))

# Imu = client.getImuData().time_stamp
# Gps = client.getGpsData().time_stamp
# Magnetometer = client.getMagnetometerData().time_stamp
# drone = client.getMultirotorState().timestamp
# difference_21 = Gps-Imu
# difference_31 = Magnetometer-Imu
# difference_32 = Magnetometer-Gps
# difference_41 = drone-Imu
# difference_42 = drone-Gps
# difference_43 = drone-Magnetometer
# print(difference_21, difference_31, difference_32, difference_41, difference_42, difference_43)
