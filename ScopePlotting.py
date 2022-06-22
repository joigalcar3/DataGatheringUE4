import airsim
import keyboard
from Occupancy_grid.ControllerTuning import ControllerTuning
from Occupancy_grid.user_input import load_user_input


if __name__ == "__main__":
    args = load_user_input()
    save_images = True
    store_location = "C:\\Users\jialv\OneDrive\\2020-2021\Thesis project\\3_Execution_phase\Simulator_images\INDI_tuning\\1.2\\C++\\Static"
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



import pickle
# store_location = "C:\\Users\jialv\OneDrive\\2020-2021\Thesis project\\3_Execution_phase\Simulator_images\INDI_tuning\\1.2\\C++\\Static"
store_location = "C:\\Users\\jialv\\OneDrive\\2020-2021\\Thesis project\\3_Execution_phase\\Simulator_images\\Debugging\\Negative_zero_correct_V_and_float_and_filter"
# store_location = "C:\\Users\\jialv\\OneDrive\\2020-2021\\Thesis project\\3_Execution_phase\\Simulator_images\\Debugging\\Negative_zero_correct_V"
filename = store_location + '\\' + 'pos_error_x-pos_error_y-pos_error_z.fig.pickle'
infile = open(filename, 'rb')
new_dict = pickle.load(infile)
infile.close()

potato = self.controller_tuning.data_gathered['yawtransferfcn']['yaw_transfer_fcn_3']
potato2 = [(potato[i]-potato[i-1])/0.00300006405 for i in range(1,len(potato))]
plt.figure(101)
plt.plot(potato2)
plt.show()

import pickle
figure_name = "C:\\Users\\jialv\\OneDrive\\2020-2021\\Thesis project\\3_Execution_phase\\Simulator_images\\Debugging\\Position_oscillation" + "\\" + "straight_vs_pos_ref.png"
pickle_name = "C:\\Users\\jialv\\OneDrive\\2020-2021\\Thesis project\\3_Execution_phase\\Simulator_images\\Debugging\\Position_oscillation" + "\\"  + "straight_vs_pos_ref.fig.pickle"
pickle.dump(fig, open(pickle_name, 'wb'))
plt.savefig(figure_name, dpi=300)


# import airsim
# from math import *
# client= airsim.MultirotorClient()
# pose = client.simGetVehiclePose()
# client.enableApiControl(True)
# pose.orientation.z_val = sin(pi/8)
# pose.orientation.w_val = cos(pi/8)
# client.setTeleportYawRef(degrees(pi/8))
# client.simSetVehiclePose(pose, True)
# client.moveOnPathAsync([airsim.Vector3r(1,1,0)], 1, 10000, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), 2, 1)