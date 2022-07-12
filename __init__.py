# User input
if __name__ == '__main__':
    # Import self created libraries
    from user_input import load_user_input
    from DataGathering import DataGathering
    from _init_paths import init_paths

    # Import Python libraries
    import sys
    from icecream import ic
    import json

    # Setup the system paths
    init_paths()

    # Load the user input
    args = load_user_input()

    # Debugging with icecream
    gettrace = getattr(sys, 'gettrace', None)
    if gettrace() and not args.PSO_tuning_switch:
        ic.enable()
    else:
        ic.disable()

    # Retrieving the start position of the drone
    location_json_file = "C:\\Users\jialv\OneDrive\Documentos\AirSim\settings.json"
    with open(location_json_file) as f:
        data = json.load(f)
    vehicle_name = list(data['Vehicles'].keys())[0]
    coord = data['Vehicles'][vehicle_name]
    start_coord = (coord['X'], coord['Y'], coord['Z'])

    data_gathering = DataGathering(vehicle_name=vehicle_name, vehicle_start_position=start_coord)
    if args.PSO_tuning_switch:
        data_gathering.pso_pid_tuning()
    else:
        data_gathering.gather_data_consecutive_runs()