import airsim
import keyboard
from Occupancy_grid.ControllerTuning import ControllerTuning
from Occupancy_grid.user_input import load_user_input


if __name__ == "__main__":
    args = load_user_input()
    data_gather_types = args.data_gather_types
    plotting_controller_signals = args.plotting_controller_signals
    client = airsim.MultirotorClient()
    controller = ControllerTuning(client, True, data_gather_types, plotting_controller_signals)

    controller.initialize_data_gathering()
    while True:
        # Manual break in the collection of data
        if keyboard.is_pressed('K'):
            print('The letter K has been pressed.')
            break

    controller.collect_data_gathered()
    controller.scope_plotting_signals()
    controller.clean_data_gathered()
