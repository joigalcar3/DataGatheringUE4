#!/usr/bin/env python
"""
Provides the code to run multiple drone agents collecting data (threads) simultaneously within the same simulation
environment.

** EXPERIMENTAL **
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Alpha"

# Imports
import os
import sys
import json
import queue
import threading
from icecream import ic

from utils import merge_flight_infos
from user_input import load_user_input
from _init_json_config import find_config_json
from Drone_flight.Data_gathering.DataGathering import DataGathering


class MultiThreadDataGathering(threading.Thread):
    """
    Class that allows to run multiple drones gathering data simultaneosly in simulation. The simulation is slowed down
    considerably as more drone agents are added.
    """
    def __init__(self, user_input, thread_ID, name, drone_name, q2, location_json):
        """
        Initializes the MultiThreadDataGathering for running multiple drones simultaneously
        :param user_input: user inputs
        :param thread_ID: the thread used for communicating with a drone
        :param name: the name of the thread
        :param drone_name: the name of the drone
        :param q2: the client queue
        :param location_json: the location of the settings json file
        """
        threading.Thread.__init__(self)
        self.user_input = user_input
        self.thread_ID = thread_ID
        self.name = name
        self.drone_name = drone_name
        self.q2 = q2
        self.location_json = location_json

    def run(self):
        """
        Method that runs the DataGathering pipeline for the drone represented in the MultiThreadDataGathering
        instantiation.
        :return: None
        """
        # Retrieve the data from the settings json file
        with open(self.location_json) as f:
            data = json.load(f)
        coord = data['Vehicles'][self.drone_name]
        start_coord = (coord['X'], coord['Y'], coord['Z'])
        print("Starting thread " + self.drone_name)
        print(self.drone_name, start_coord)

        # Run the data gathering pipeline
        data_gatherer = DataGathering(data, self.user_input, vehicle_name=self.drone_name,
                                      vehicle_start_position=start_coord)
        data_gatherer.gather_data_single_run()
        if self.q2.empty():
            self.q2.put(data_gatherer.drone_flight.client)


if __name__ == "__main__":
    # Debugging code
    gettrace = getattr(sys, 'gettrace', None)
    if gettrace():
        ic.enable()
    else:
        ic.disable()
    print(os.getcwd())

    # User input
    args = load_user_input()

    # Obtain the location of the Airsim json configuration file and retrieve the data
    location_json_file, data = find_config_json(args)

    # Obtain the drones that will be flying
    drones_available = list(data['Vehicles'].keys())
    flight_info_folder = "Flight_info"
    existing_datasets = os.listdir(flight_info_folder)

    # Creation of threads
    client_Queue = queue.Queue(1)
    number_drones_available = len(drones_available)
    it = 2
    for run in range(args.number_runs):
        threads = []
        for number in range(number_drones_available):
            thread_number = number_drones_available*run+number+1  # create different thread # for each drone in each run
            print("thread_number = ", thread_number)
            thread = MultiThreadDataGathering(args, thread_number, str(thread_number), drones_available[number],
                                              client_Queue, location_json_file)
            threads.append(thread)

        # Initialize every thread
        for number in range(number_drones_available):
            threads[number].start()

        for t in threads:
            t.join()
        client_Queue.get().reset(True)

        # Run all the threads and merge the data gathered by all the flights
        it = merge_flight_infos(existing_datasets, flight_info_folder, run, it)
