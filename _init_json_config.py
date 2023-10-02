#!/usr/bin/env python
"""
Provides the function that returns the location of the Airsim json settings file.
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
import json


def find_config_json(args_input):
    """
    Obtain the json configuration file that contains information regarding the number of drones and their names
    :param args_input: the inputs from the user input file
    :return: file path to the json settings file and the contents of the file
    """
    try:
        location_json_file = args_input.json_config_loc
        with open(location_json_file) as f:
            data = json.load(f)
        print("The json config file is at the provided address.")
    except:
        try:
            location_json_file = "C:\\Users\\jdealvearcarde\\AirSim\\settings.json"  # Location in Jasper's computer
            with open(location_json_file) as f:
                data = json.load(f)
            print("Jasper's PC json config file location was used.")
        except:
            try:
                location_json_file = "C:\\Users\jialv\OneDrive\Documentos\AirSim\settings.json"  # Location in personal computer
                with open(location_json_file) as f:
                    data = json.load(f)
                print("Jose's laptop json config file location was used.")
            except:
                raise ValueError("The json config address could not be found.")

    return location_json_file, data
