import json


def find_config_json(args_input):
    """
    Obtain the json configuration file that contains information regarding the number of drones and their names
    :param args_input: the inputs from the user input file
    :return:
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
