import airsim
import keyboard
from Occupancy_grid.ControllerTuning import ControllerTuning
from Occupancy_grid.user_input import load_user_input


if __name__ == "__main__":
    args = load_user_input()
    save_images = True
    store_location = "C:\\Users\jialv\OneDrive\\2020-2021\Thesis project\\3_Execution_phase\Simulator_images\INDI_tuning\\1.1\\C_Matlab_comparison\\Static\\C++\\dummy"
    data_gather_types = args.data_gather_types
    plotting_controller_signals = args.plotting_controller_signals
    plotting_controller_signals_aeo = args.plotting_controller_signals_aeo
    client = airsim.MultirotorClient()
    controller = ControllerTuning(client, True, data_gather_types,
                                  plotting_controller_signals_aeo=plotting_controller_signals_aeo,
                                  plotting_controller_signals=plotting_controller_signals)

    controller.initialize_data_gathering()
    while True:
        # Manual break in the collection of data
        if keyboard.is_pressed('K'):
            print('The letter K has been pressed.')
            break

    controller.collect_data_gathered()
    controller.scope_plotting_signals(save_images, store_location)
    controller.clean_data_gathered()
