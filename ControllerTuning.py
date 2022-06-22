import scipy.integrate as integrate
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pickle


class ControllerTuning:
    def __init__(self, client, controller_tuning_switch, data_gather_types, plotting_controller_signals_aeo,
                 plotting_controller_signals, vehicle_name=''):
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
        self.plotting_controller_signals_aeo = plotting_controller_signals_aeo  # plotting signals against each other
        self.vehicle_name = vehicle_name
        self.data_gathered = {}
        self.figure_number = 1
        self.colour_list = ['r-', 'b-', 'g-', 'm-', 'y-']
        self.number_data_points = None

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
            self.client.setPlotDataCollectionActivation(True)

    def collect_data_gathered(self):
        """
        Collect the data of the signals specified within data_gather_types
        :return:
        """
        if self.controller_tuning_switch:
            self.client.setPlotDataCollectionActivation(False)
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

    def scope_plotting_signals(self, save_images=False, store_location=""):
        """
        Plot the signals and group of signals specified within plotting_controller_signals
        :return:
        """
        if self.controller_tuning_switch and self.plotting_controller_signals:
            for i in range(len(self.plotting_controller_signals)):
                signals = self.plotting_controller_signals[i]
                package = {}
                for j in range(len(signals)):
                    name, data = self.retrieve_name_data(signals[j])
                    if i == 0 and j == 0:
                        self.number_data_points = len(data)
                        package[name] = data
                    else:
                        package[name] = data[:self.number_data_points]
                self.scope_plotting_signal(package, save_images, store_location)

            for i in range(len(self.plotting_controller_signals_aeo)):
                lines = self.plotting_controller_signals_aeo[i]
                package_lines = []
                names_lines = []
                for j in range(len(lines)):
                    signal = lines[j]
                    package_signals = []
                    names_signals = []
                    for k in range(len(signal)):
                        name, data = self.retrieve_name_data(signal[k])
                        names_signals.append(name)
                        package_signals.append(data[:self.number_data_points])
                    package_lines.append(package_signals)
                    names_lines.append(names_signals)
                self.nD_signal_plotting(package_lines, names_lines, save_images, store_location)
        plt.show()

    def retrieve_name_data(self, signal):
        complete_name = signal.split('.')
        if len(complete_name) == 1:
            name = complete_name[0]
            data = self.data_gathered[name]
        else:
            name = complete_name[-1]
            data = self.data_gathered[complete_name[0]][name]
        return name, data

    def scope_plotting_signal(self, signals, save_images, store_location):
        """
        Method to plot a provided signal or signals within the same figure for comparison
        :return:
        """
        names = list(signals.keys())
        title = '-'.join(names)

        fig = plt.figure(self.figure_number)
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
        if save_images:
            figure_name = store_location + "\\" + title + ".png"
            pickle_name = store_location + "\\" + title + ".fig.pickle"
            pickle.dump(fig, open(pickle_name, 'wb'))
            plt.savefig(figure_name, dpi=300)

    def nD_signal_plotting(self, signals, names, save_images, store_location):
        signal_dimensionality = []
        number_signals = len(signals)
        for i in range(number_signals):
            signal_dimensionality.append(len(signals[i]))
        max_signal_dimensionality = max(signal_dimensionality)

        title = ""
        x_label = ""
        y_label = ""
        if max_signal_dimensionality == 2:
            fig = plt.figure(self.figure_number)
        elif max_signal_dimensionality == 3:
            fig = plt.figure(self.figure_number)
            ax = plt.axes(projection='3d')
            z_label = ""
        self.figure_number += 1
        for i in range(number_signals):
            x_points = signals[i][0]
            y_points = signals[i][1]
            x_label += names[i][0]
            y_label += names[i][1]
            name = names[i][0] + "-" + names[i][1]
            if max_signal_dimensionality == 2:
                plt.plot(x_points, y_points, self.colour_list[i], label=name)
            elif max_signal_dimensionality == 3:
                z_points = signals[i][2]
                z_label += names[i][2]
                name += "-" + names[i][2]
                ax.plot3D(x_points, y_points, z_points, self.colour_list[i], label=name)
            title += "(" + name + ")"
            if (i + 1) != number_signals:
                title = title + " vs "
                x_label += " - "
                y_label += " - "
        if max_signal_dimensionality == 2:
            plt.title(title)
            plt.xlabel(x_label)
            plt.ylabel(y_label)
            plt.grid(True)
            plt.legend()
            if save_images:
                figure_name = store_location + "\\" + title + ".png"
                pickle_name = store_location + "\\" + title + ".fig.pickle"
                pickle.dump(fig, open(pickle_name, 'wb'))
                plt.savefig(figure_name, dpi=300)
        elif max_signal_dimensionality == 3:
            ax.set_title(title)
            ax.set_xlabel(x_label)
            ax.set_ylabel(y_label)
            ax.set_zlabel(z_label)
            ax.invert_zaxis()
            ax.legend()
            if save_images:
                figure_name = store_location + "\\" + title + ".png"
                pickle_name = store_location + "\\" + title + ".fig.pickle"
                pickle.dump(fig, open(pickle_name, 'wb'))
                plt.savefig(figure_name, dpi=300)

    def reset(self):
        """
        Clean the C++ variables that store the data and reset the parameters from the class
        :return:
        """
        self.clean_data_gathered()
        self.data_gathered = {}
        self.figure_number = 1
