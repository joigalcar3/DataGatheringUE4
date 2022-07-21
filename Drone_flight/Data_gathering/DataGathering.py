import sys
from icecream import ic
from user_input import load_user_input
from _init_json_config import find_config_json
from Drone_flight.DroneFlight import DroneFlight
from pyswarm import pso
import airsim


class DataGathering:
    """
    Class that allows the data gathering of multiple files
    """
    def __init__(self, data, user_input, vehicle_name='', vehicle_start_position=None):
        self.user_input = user_input
        self.number_runs = self.user_input.number_runs
        self.flight_altitudes = self.user_input.flight_altitudes
        self.constant_altitude_iterations = self.user_input.constant_altitude_iterations

        clock_speed = data['ClockSpeed']
        sample_rates = {key: value*clock_speed for key, value in self.user_input.sample_rates.items()}

        # Create a Drone
        self.drone_flight = DroneFlight(self.user_input,
                                        sample_rates=sample_rates, clock_speed=clock_speed,
                                        vehicle_name=vehicle_name,
                                        vehicle_start_position=vehicle_start_position)

    def gather_data_consecutive_runs(self):
        """
        Prepares and launches the required flights according to the number of runs specified.
        :return:
        """
        self.drone_flight.reset(False)
        for run in range(self.number_runs):
            if run % self.constant_altitude_iterations == 0:
                activate_map_extraction = True
            else:
                activate_map_extraction = False
            _ = self.drone_flight.run(navigation_type=self.user_input.navigation_type, start_point=self.user_input.start,
                                      goal_point=self.user_input.goal, min_h=self.flight_altitudes[0],
                                      max_h=self.flight_altitudes[1], activate_reset=True,
                                      activate_map_extraction=activate_map_extraction)

    def gather_data_single_run(self):
        """
        Prepares and launches a single run or drone flight
        :return:
        """
        self.drone_flight.run(navigation_type=self.user_input.navigation_type, start_point=self.user_input.start,
                              goal_point=self.user_input.goal, min_h=self.flight_altitudes[0],
                              max_h=self.flight_altitudes[1], activate_reset=False)

    def pso_pid_function(self, x):
        """
        Prepares and launches the required flights according to the number of runs specified in order to tune the PID
        controller with PSO.
        :return:
        """
        self.drone_flight.reset(True)
        V_x_gains = airsim.PIDGains(x[0], x[1], 0)
        V_y_gains = airsim.PIDGains(x[2], x[3], 0)
        V_z_gains = airsim.PIDGains(x[4], x[5], 0)
        angle_x_gains = airsim.PIDGains(x[6], x[7], 0)
        angle_y_gains = airsim.PIDGains(x[8], x[9], 0)
        angle_z_gains = airsim.PIDGains(x[10], x[11], 0)

        self.drone_flight.client.setVelocityControllerGains(
            airsim.VelocityControllerGains(x_gains=V_x_gains, y_gains=V_y_gains, z_gains=V_z_gains))
        self.drone_flight.client.setAngleLevelControllerGains(
            airsim.AngleLevelControllerGains(roll_gains=angle_x_gains, pitch_gains=angle_y_gains,
                                             yaw_gains=angle_z_gains))

        total_error = self.drone_flight.run(navigation_type=self.user_input.navigation_type, start_point=self.user_input.start,
                                            goal_point=self.user_input.goal, min_h=self.flight_altitudes[0],
                                            max_h=self.flight_altitudes[1], activate_reset=True)
        return total_error
        # for run in range(self.number_runs):
        #     total_error = self.drone_flight.run(navigation_type=self.user_input.navigation_type, start_point=self.user_input.start,
        #                           goal_point=self.user_input.goal, min_h=self.flight_altitudes[0],
        #                           max_h=self.flight_altitudes[1], activate_reset=True)

    def pso_pid_tuning(self):
        """
        Method which carries out the PSO optimisation of the PID controller values
        :return:
        """
        ub = [4] * 12  # upper bound
        lb = [0] * 12  # lower bound
        xopt_CNN, fopt_CNN = pso(self.pso_pid_function, lb, ub, debug=True,
                                 swarmsize=100, maxiter=20)  # PSO optimisation
        # print("The best PID parameters are as follows:\n"
        #       "x_gains = {} \n"
        #       "y_gains = {} \n"
        #       "z_gains = {} \n".format(xopt_CNN[:3], xopt_CNN[3:6], xopt_CNN[6:]))
        print("The best PID parameters are as follows:\n"
              "V_x_gains = {} \n"
              "V_y_gains = {} \n"
              "V_z_gains = {} \n"
              "angle_x_gains = {} \n"
              "angle_y_gains = {} \n"
              "angle_z_gains = {} \n"
              .format(xopt_CNN[:2] + [0], xopt_CNN[2:4] + [0], xopt_CNN[4:6] + [0], xopt_CNN[6:8] + [0],
                      xopt_CNN[8:10] + [0], xopt_CNN[10:12] + [0]))
        print("The cost function value is: {}".format(fopt_CNN))


if __name__ == "__main__":
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

    vehicle_name = list(data['Vehicles'].keys())[0]
    coord = data['Vehicles'][vehicle_name]
    start_coord = (coord['X'], coord['Y'], coord['Z'])

    data_gathering = DataGathering(data, args, vehicle_name=vehicle_name, vehicle_start_position=start_coord)
    if args.PSO_tuning_switch:
        data_gathering.pso_pid_tuning()
    else:
        data_gathering.gather_data_consecutive_runs()
