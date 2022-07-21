# User input
if __name__ == '__main__':
    # Setup the system paths
    from _init_paths import init_paths
    init_paths()

    # Import self created libraries
    from user_input import load_user_input
    from Drone_flight.Data_gathering.DataGathering import DataGathering

    from _init_json_config import find_config_json

    # Import Python libraries
    import sys
    from icecream import ic
    import matplotlib as mpl

    # Set up the plotting backend
    mpl.use('TKAgg')

    # Load the user input
    args = load_user_input()

    # Obtain the location of the Airsim json configuration file and retrieve the data
    location_json_file, data = find_config_json(args)

    # Debugging with icecream
    gettrace = getattr(sys, 'gettrace', None)
    if gettrace() and not args.PSO_tuning_switch:
        ic.enable()
    else:
        ic.disable()

    # Retrieving the start position of the drone
    vehicle_name = list(data['Vehicles'].keys())[0]
    coord = data['Vehicles'][vehicle_name]
    start_coord = (coord['X'], coord['Y'], coord['Z'])

    data_gathering = DataGathering(data, args, vehicle_name=vehicle_name, vehicle_start_position=start_coord)
    if args.PSO_tuning_switch:
        data_gathering.pso_pid_tuning()
    else:
        data_gathering.gather_data_consecutive_runs()
