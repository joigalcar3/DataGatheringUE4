import sys
import _init_paths
from icecream import ic
from user_input import load_user_input
from DroneFlight import DroneFlight
from math import ceil
from pyswarm import pso
import airsim


class DataGathering:
    """
    Class that allows the data gathering of multiple files
    """
    def __init__(self, vehicle_name='', vehicle_start_position=None):
        self.args = load_user_input()
        self.number_runs = self.args.number_runs
        self.navigation_type = self.args.navigation_type
        self.flight_altitudes = self.args.flight_altitudes
        self.vehicle_name = vehicle_name
        self.vehicle_start_position = vehicle_start_position

        location_json_file = "C:\\Users\jialv\OneDrive\Documentos\AirSim\settings.json"
        with open(location_json_file) as f:
            data = json.load(f)
        clock_speed = data['ClockSpeed']
        sample_rates = {key: value*clock_speed for key, value in self.args.sample_rates.items()}

        # Create a Drone
        self.drone_flight = DroneFlight(self.args.altitude_m, self.args.altitude_range_m, self.args.cell_size_m,
                                        self.args.ue4_airsim_conversion_units, self.args.robot_radius,
                                        self.args.sensors_lst,
                                        self.args.cameras_info,
                                        sample_rates=sample_rates, clock_speed=clock_speed,
                                        min_flight_distance_m=self.args.min_flight_distance_m,
                                        max_flight_distance_m=self.args.max_flight_distance_m,
                                        saved_vertices_filename=self.args.saved_vertices_filename,
                                        update_saved_vertices=self.args.update_saved_vertices, plot2D=self.args.plot2D,
                                        plot3D=self.args.plot3D, PID_tuning=self.args.PID_tuning,
                                        vehicle_name=self.vehicle_name,
                                        vehicle_start_position=self.vehicle_start_position, smooth=self.args.smooth,
                                        failure_types=self.args.failure_types,
                                        activate_take_off=self.args.activate_take_off)

    def gather_data_consecutive_runs(self):
        """
        Prepares and launches the required flights according to the number of runs specified.
        :return:
        """
        self.drone_flight.reset(True)
        for run in range(self.number_runs):
            self.drone_flight.run(navigation_type=self.navigation_type, start_point=self.args.start,
                                  goal_point=self.args.goal, min_h=self.flight_altitudes[0],
                                  max_h=self.flight_altitudes[1], activate_reset=True)

    def pso_pid_function(self, x):
        """
        Prepares and launches the required flights according to the number of runs specified in order to tune the PID
        controller with PSO.
        :return:
        """
        self.drone_flight.reset(True)
        x_gains = airsim.PIDGains(x[0], x[1], x[2])
        y_gains = airsim.PIDGains(x[3], x[4], x[5])
        z_gains = airsim.PIDGains(x[6], x[7], x[8])
        self.drone_flight.client.setVelocityControllerGains(
            airsim.VelocityControllerGains(x_gains=x_gains, y_gains=y_gains, z_gains=z_gains))
        total_error = self.drone_flight.run(navigation_type=self.navigation_type, start_point=self.args.start,
                                            goal_point=self.args.goal, min_h=self.flight_altitudes[0],
                                            max_h=self.flight_altitudes[1], activate_reset=True)
        return total_error
        # for run in range(self.number_runs):
        #     total_error = self.drone_flight.run(navigation_type=self.navigation_type, start_point=self.args.start,
        #                           goal_point=self.args.goal, min_h=self.flight_altitudes[0],
        #                           max_h=self.flight_altitudes[1], activate_reset=True)

    def pso_pid_tuning(self):
        """
        Method which carries out the PSO optimisation of the PID controller values
        :return:
        """
        ub = [4] * 9  # upper bound
        lb = [0] * 9  # lower bound
        xopt_CNN, fopt_CNN = pso(self.pso_pid_function, lb, ub, debug=True,
                                 swarmsize=100, maxiter=20)  # PSO optimisation
        print("The best PID parameters are as follows:\n"
              "x_gains = {} \n"
              "y_gains = {} \n"
              "z_gains = {} \n".format(xopt_CNN[:3], xopt_CNN[3:6], xopt_CNN[6:]))
        print("The cost function value is: {}".format(fopt_CNN))

    def gather_data_single_run(self):
        """
        Prepares and launches a single run or drone flight
        :return:
        """
        self.drone_flight.run(navigation_type=self.navigation_type, start_point=self.args.start,
                              goal_point=self.args.goal, min_h=self.flight_altitudes[0],
                              max_h=self.flight_altitudes[1], activate_reset=False)


if __name__ == "__main__":
    import json

    args = load_user_input()
    # Debugging with icecream
    gettrace = getattr(sys, 'gettrace', None)
    if gettrace() and not args.PID_tuning:
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
    if args.PID_tuning:
        data_gathering.pso_pid_tuning()
    else:
        data_gathering.gather_data_consecutive_runs()
