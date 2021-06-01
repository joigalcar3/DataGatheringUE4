from Occupancy_grid.user_input import load_user_input
from Occupancy_grid.DroneFlight import DroneFlight
import random


class DataGathering:
    """
    Class that allows the data gathering of multiple files
    """
    def __init__(self):
        self.args = load_user_input()
        self.number_runs = self.args.number_runs
        self.navigation_type = self.args.navigation_type

        # Create a Drone
        self.drone_flight = DroneFlight(self.args.altitude_m, self.args.altitude_range_m, self.args.cell_size_m,
                                        self.args.ue4_airsim_conversion_units, self.args.robot_radius,
                                        self.args.sensors_lst,
                                        self.args.cameras_info,
                                        self.args.sample_rate, min_flight_distance_m=self.args.min_flight_distance_m,
                                        saved_vertices_filename=self.args.saved_vertices_filename,
                                        update_saved_vertices=self.args.update_saved_vertices, plot2D=self.args.plot2D,
                                        plot3D=self.args.plot3D)

    def gather_data_consecutive_runs(self):
        """
        Prepares and launches the required flights according to the number of runs specified.
        :return:
        """
        for run in range(self.number_runs):
            h = -random.randint(1, 5)
            print("Altitude: ", h)
            self.drone_flight.extract_occupancy_map(h)
            self.drone_flight.navigate_drone_grid(navigation_type=self.navigation_type,
                                                  start_point=self.args.start, goal_point=self.args.goal)
            self.drone_flight.teleport_drone_start()
            self.drone_flight.take_off()
            self.drone_flight.fly_trajectory()
            self.drone_flight.obtain_sensor_data()
            self.drone_flight.client.reset()


if __name__ == "__main__":
    data_gathering = DataGathering()
    data_gathering.gather_data_consecutive_runs()
