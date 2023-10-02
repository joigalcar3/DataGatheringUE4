#!/usr/bin/env python
"""
Provides the tool to scope vehicle signals and computes the position error of the vehicle for its PID controller tuning.

It create a functionality similar to the scoping function within Matlab in which the user can see at the end of the
simulation the resulting signals for position, velocity, acceleration, etc.
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Stable"

import os
import pickle
import matplotlib.pyplot as plt
import scipy.integrate as integrate


class ControllerTuning:
    """
    Tool that collects data from the simulation and plots it for controller tuning purposes and for understanding the
    behaviour of the drone in simulation. It is similar to the scoping function in Matlab. Additionally, it has
    a method which computes the position error of the vehicle for controller tuning.
    """
    def __init__(self, user_input, client, controller_tuning_switch, vehicle_name=''):
        """
        Initialises the class in charge of collecting data from the simulation and plot it for controller tuning
        purposes
        :param user_input: the input provided by the user
        :param client: the AirSim client
        :param controller_tuning_switch: whether the controller is going to be tuned and the methods need to be executed
        :param vehicle_name: the name of the vehicle
        """
        self.controller_tuning_switch = controller_tuning_switch
        self.client = client
        self.data_gather_types = user_input.data_gather_types
        self.plotting_controller_signals = user_input.plotting_controller_signals
        self.plotting_controller_signals_aeo = user_input.plotting_controller_signals_aeo  # plotting signals against each other
        self.vehicle_name = vehicle_name
        self.data_gathered = {}
        self.figure_number = 1
        self.colour_list = ['r-', 'b-', 'g-', 'm-', 'y-']
        self.number_data_points = None

        self.save_scope_images = user_input.save_scope_images
        self.scope_images_remote_store_location = user_input.scope_images_remote_store_location
        self.scope_images_store_location = user_input.scope_images_store_location

    def initialize_data_gathering(self):
        """
        Initialize the gathering of the data signals specified within data_gather_types
        :return: None
        """
        if self.controller_tuning_switch:
            for i in range(len(self.data_gather_types)):
                name_func = 'set' + self.data_gather_types[i].capitalize() + 'Activation'
                getattr(self.client, name_func)(True, vehicle_name=self.vehicle_name)
            self.client.setPlotDataCollectionActivation(True)

    def collect_data_gathered(self):
        """
        Collect the data of the signals specified within data_gather_types
        :return: None
        """
        if self.controller_tuning_switch:
            self.client.setPlotDataCollectionActivation(False)
            for i in range(len(self.data_gather_types)):
                name_func = 'get' + self.data_gather_types[i].capitalize() + 'StoredDataVec'
                output = getattr(self.client, name_func)(vehicle_name=self.vehicle_name)
                self.data_gathered[self.data_gather_types[i]] = output

    def clean_data_gathered(self):
        """
        Clean the C++ variables that store the data of the signals specified within data_gather_types such that the
        arrays are empty for a future run.
        :return: None
        """
        if self.controller_tuning_switch:
            for i in range(len(self.data_gather_types)):
                name_func = 'clean' + self.data_gather_types[i].capitalize() + 'StoredData'
                getattr(self.client, name_func)(vehicle_name=self.vehicle_name)

    def tuning_cost_function(self, collision_type, path):
        """
        Method which computes the cost function used for the tuning of the PID controller.
        :param collision_type: the type of collision that has prompted the end of the flight
        :param path: the point coordinates that the drone should follow
        :return: the total position error
        """
        if self.controller_tuning_switch:
            self.collect_data_gathered()
            self.clean_data_gathered()

            # In the case of a failure, the total error is considered infinite (unacceptable)
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

            # Compute the error in the horizontal and vertical plains
            error_xy = abs(integrate.trapezoid(path_y, path_x) - integrate.trapezoid(position["positions_y"],
                                                                                     position["positions_x"]))

            error_z = abs(integrate.trapezoid(path_y, path_z) - integrate.trapezoid(position["positions_y"],
                                                                                    position["positions_z"]))

            # The total error is the sum of the horizontal and vertical errors. The vertical error is weighted by a
            # factor of 2.
            total_error = error_xy + 2 * error_z

            return total_error

    def scope_plotting_signals(self):
        """
        Plot the signals and group of signals specified within plotting_controller_signals
        :return: None
        """
        if self.controller_tuning_switch and self.plotting_controller_signals:
            # Iterate over all the signals that should be plotted
            for i in range(len(self.plotting_controller_signals)):
                signals = self.plotting_controller_signals[i]
                package = {}
                for j in range(len(signals)):
                    name, data = self.retrieve_name_data(signals[j])  # retrieve name of signal and the captured data

                    # Compute the number of data points from the first signal
                    if i == 0 and j == 0:
                        self.number_data_points = len(data)
                        package[name] = data
                    else:
                        package[name] = data[:self.number_data_points]

                # Plot each of the signals. If they are within the same sublist, they are plotted together
                self.scope_plotting_signal(package)

            # Similar process is followed when multiple signals are plotted in the same figure
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

                # Plotting signals in a higher dimensional space, like 3D
                self.nD_signal_plotting(package_lines, names_lines)
        plt.show()

    def retrieve_name_data(self, signal):
        """
        Method which separates the signals provided by the user by the ".". The last element of the split signal
        name provided by the user is the name used for plotting. The captured data during simulation is also returned.
        :param signal: the name of the user provided signal
        :return: the name of the signal and the data captured
        """
        complete_name = signal.split('.')
        if len(complete_name) == 1:
            name = complete_name[0]
            data = self.data_gathered[name]
        else:
            name = complete_name[-1]
            data = self.data_gathered[complete_name[0]][name]
        return name, data

    def scope_plotting_signal(self, signals):
        """
        Method to plot a provided list of signal separated or together in the sme figure if they are part of the same
        sublist.
        :param signals: dictionary with the signal names an the values to be plotted
        :return: None
        """
        # Obtain the signal names and create plot title out of them
        names = list(signals.keys())
        title = '-'.join(names)

        # Plot the signals
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

        # If the user expressed the need, save the figure in the desired folder
        if self.save_scope_images:
            scope_store_folder = os.path.join(self.scope_images_remote_store_location,
                                              "Saved_scope_signals", self.scope_images_store_location)
            isExist = os.path.exists(scope_store_folder)
            if not isExist:
                os.makedirs(scope_store_folder)

            # Stored the figure as a png and as a pickle in the case that the user wants to interact with it later
            figure_name = scope_store_folder + "\\" + title + ".png"
            pickle_name = scope_store_folder + "\\" + title + ".fig.pickle"
            pickle.dump(fig, open(pickle_name, 'wb'))
            plt.savefig(figure_name, dpi=300)

    def nD_signal_plotting(self, signals, names):
        """
        Method to plot a signal in a higher dimensional space than 1D. For example, the trajectory that the vehicle
        should follow in 3D and the actual trajectory
        :param signals: dictionary with the signal names as keys and arrays as values
        :param names: name given to the line plotted in higher dimensional space
        :return: None
        """
        # Compute the number of signals to be plotted and the dimensionality of the plot
        signal_dimensionality = []
        number_signals = len(signals)
        for i in range(number_signals):
            signal_dimensionality.append(len(signals[i]))
        max_signal_dimensionality = max(signal_dimensionality)

        # Different actions depending on the dimensionality of the plot
        title = ""
        x_label = ""
        y_label = ""
        if max_signal_dimensionality == 2:
            fig = plt.figure(self.figure_number)
        elif max_signal_dimensionality == 3:
            fig = plt.figure(self.figure_number)
            ax = plt.axes(projection='3d')
            z_label = ""

        # Create the plot
        self.figure_number += 1
        for i in range(number_signals):
            x_points = signals[i][0]
            y_points = signals[i][1]
            x_label += names[i][0]
            y_label += names[i][1]
            name = names[i][0] + "-" + names[i][1]
            if max_signal_dimensionality == 2:  # 2D
                plt.plot(x_points, y_points, self.colour_list[i], label=name)
            elif max_signal_dimensionality == 3:  # 3D
                z_points = signals[i][2]
                z_label += names[i][2]
                name += "-" + names[i][2]
                ax.plot3D(x_points, y_points, z_points, self.colour_list[i], label=name)
            title += "(" + name + ")"

            # Iteratively compose title and labels
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

            # If the user expressed the need, save the figure in the desired folder
            if self.save_scope_images:
                scope_store_folder = os.path.join(self.scope_images_remote_store_location,
                                                  "Saved_scope_signals", self.scope_images_store_location)
                isExist = os.path.exists(scope_store_folder)
                if not isExist:
                    os.makedirs(scope_store_folder)

                # Stored the figure as a png and as a pickle in the case that the user wants to interact with it later
                figure_name = scope_store_folder + "\\" + title + ".png"
                pickle_name = scope_store_folder + "\\" + title + ".fig.pickle"
                pickle.dump(fig, open(pickle_name, 'wb'))
                plt.savefig(figure_name, dpi=300)
        elif max_signal_dimensionality == 3:
            ax.set_title(title)
            ax.set_xlabel(x_label)
            ax.set_ylabel(y_label)
            ax.set_zlabel(z_label)
            ax.invert_zaxis()
            ax.legend()

            # If the user expressed the need, save the figure in the desired folder
            if self.save_scope_images:
                scope_store_folder = os.path.join(self.scope_images_remote_store_location,
                                                  "Saved_scope_signals", self.scope_images_store_location)
                isExist = os.path.exists(scope_store_folder)
                if not isExist:
                    os.makedirs(scope_store_folder)

                # Stored the figure as a png and as a pickle in the case that the user wants to interact with it later
                figure_name = scope_store_folder + "\\" + title + ".png"
                pickle_name = scope_store_folder + "\\" + title + ".fig.pickle"
                pickle.dump(fig, open(pickle_name, 'wb'))
                plt.savefig(figure_name, dpi=300)

    def reset(self):
        """
        Clean the C++ variables that store the data and reset the parameters from the class
        :return: None
        """
        self.clean_data_gathered()
        self.data_gathered = {}
        self.figure_number = 1
