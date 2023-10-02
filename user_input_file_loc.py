#!/usr/bin/env python
"""
Provides the file path to folders where data could be found or stored.

It should be modified by the user to the right file paths before use
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Stable"


def load_user_input_file_loc(parser):
    """
    Function that loads the file paths to locations where data can be found or will be stored
    :param parser: user_input parser
    :return: the input parser
    """
    # Arguments related to the occupancy map generation
    parser.add_argument('--saved_vertices_filename', type=str, default='Blocks_object_points',
                        # 'object_points', 'SunTemple_object_points', CoenCity_Object_points
                        help='Name of cloud points file.')

    # Arguments related to the sensor data collection
    parser.add_argument('--sensors_remote_storage_location',
                        default="D:\\AirSim simulator\\DataGathering Python API\\multirotor\\Occupancy_grid",
                        help='Location where the sensor information should be stored. If None, it will be stored'
                             'within the "Occupancy_grid\\Drone_flight\\Data_gathering" folder.')

    # Arguments related to the failure factory
    parser.add_argument('--flight_info_remote_storage_location',
                        default='D:\\AirSim simulator\\DataGathering Python API\\multirotor\\Occupancy_grid',
                        help='Location where the flight information should be stored. If None, it will be stored'
                             'within the "Occupancy_grid\\Drone_flight\\Failure_injection" folder.')

    # Arguments for controller tuning
    parser.add_argument('--scope_images_remote_store_location', type=str, default="",
                        help='The path to the folder where the scope images are stored.')
    parser.add_argument('--scope_images_store_location', type=str, default="",
                        help='The name of the folder where the scope images of the current run should be stored.'
                             'This folder is located within "scope_images_remote_store_location".')

    # Arguments os search
    parser.add_argument('--json_config_loc', type=str,
                        default="C:\\Users\\jialv\\OneDrive\\Documentos\\AirSim\\settings.json",
                        help='Location of the json configuration file for AirSim.')

    return parser
