import scipy.integrate as integrate
import matplotlib.pyplot as plt


class ControllerTuning:
    def __init__(self, client, controller_tuning_switch, data_gather_types, plotting_controller_signals, vehicle_name=''):
        """
        Initialises the class in charge of collecting data from the simulation and plot it for controller tuning
        purposes
        :param controller_tuning_switch: whether the controller is going to be tuned and the methods need to be executed
        :param data_gather_types: what data is going to be gathered
        :param plotting_controller_signals: which groups of signals are going to be plotted
        :param vehicle_name: the name of the vehicle
        """
        self.controller_tuning_switch = controller_tuning_switch
        self.client = client
        self.data_gather_types = data_gather_types
        self.plotting_controller_signals = plotting_controller_signals
        self.vehicle_name = vehicle_name
        self.data_gathered = {}
        self.figure_number = 1
        self.colour_list = ['r-', 'b-', 'g-', 'm-', 'y-']

    def initialize_data_gathering(self):
        """
        Initialize the gathering of the data signals specified within data_gather_types
        :param client: the AirSim client used
        :return:
        """
        if self.controller_tuning_switch:
            for i in range(len(self.data_gather_types)):
                name_func = 'set' + self.data_gather_types[i].capitalize() + 'Activation'
                getattr(self.client, name_func)(True, vehicle_name=self.vehicle_name)

    def collect_data_gathered(self):
        """
        Collect the data of the signals specified within data_gather_types
        :return:
        """
        if self.controller_tuning_switch:
            for i in range(len(self.data_gather_types)):
                name_func = 'get' + self.data_gather_types[i].capitalize() + 'StoredDataVec'
                output = getattr(self.client, name_func)(vehicle_name=self.vehicle_name)
                self.data_gathered[self.data_gather_types[i]] = output

    def clean_data_gathered(self):
        """
        Clean the C++ variables that store the data of the signals specified within data_gather_types
        :return:
        """
        if self.controller_tuning_switch:
            for i in range(len(self.data_gather_types)):
                name_func = 'clean' + self.data_gather_types[i].capitalize() + 'StoredData'
                getattr(self.client, name_func)(vehicle_name=self.vehicle_name)

    def tuning_cost_function(self, collision_type, path):
        """
        Method which computes the cost function used for the tuning of the PID controller.
        :return:
        """
        if self.controller_tuning_switch:
            self.collect_data_gathered()
            self.clean_data_gathered()

            print(collision_type)
            if collision_type:
                total_error = float("inf")
                return total_error

            position = self.data_gathered['position']

            path_x = []
            path_y = []
            path_z = []
            for i in range(len(path)):
                path_x.append(path[i].x_val)
                path_y.append(path[i].y_val)
                path_z.append(path[i].z_val)
            # self.controller_tuning_plotting(path_x, path_y, path_z, position, PWMs)

            error_xy = abs(integrate.trapezoid(path_y, path_x) - integrate.trapezoid(position["positions_y"],
                                                                                     position["positions_x"]))

            error_z = abs(integrate.trapezoid(path_y, path_z) - integrate.trapezoid(position["positions_y"],
                                                                                    position["positions_z"]))

            total_error = error_xy + 2 * error_z

            return total_error

    def scope_plotting_signals(self):
        """
        Plot the signals and group of signals specified within plotting_controller_signals
        :return:
        """
        if self.controller_tuning_switch and self.plotting_controller_signals:
            for i in range(len(self.plotting_controller_signals)):
                signals = self.plotting_controller_signals[i]
                package = {}
                for j in range(len(signals)):
                    complete_name = signals[j].split('.')
                    if len(complete_name) == 1:
                        name = complete_name[0]
                        data = self.data_gathered[name]
                    else:
                        name = complete_name[-1]
                        data = self.data_gathered[complete_name[0]][name]
                    package[name] = data
                self.scope_plotting_signal(package)
        plt.show()

    def scope_plotting_signal(self, signals):
        """
        Method to plot a provided signal or signals within the same figure for comparison
        :return:
        """
        names = list(signals.keys())
        title = '-'.join(names)

        plt.figure(self.figure_number)
        self.figure_number += 1
        for i in range(len(names)):
            y_points = signals[names[i]]
            x_points = range(len(y_points))
            plt.plot(x_points, y_points, self.colour_list[i], label=names[i])
        plt.title(title)
        plt.xlabel("Index")
        plt.ylabel("Measured_value")
        plt.grid(True)
        plt.legend()

    def reset(self):
        """
        Clean the C++ variables that store the data and reset the parameters from the class
        :return:
        """
        self.clean_data_gathered()
        self.data_gathered = {}
        self.figure_number = 1