import threading
import queue
import sys
import os
from icecream import ic
import json

from Occupancy_grid.DataGathering import DataGathering
from Occupancy_grid.user_input import load_user_input
from Occupancy_grid.merge_flight_infos import merge_flight_infos


class MultiThreadDataGathering(threading.Thread):
    def __init__(self, thread_ID, name, drone_name, q2, location_json):
        threading.Thread.__init__(self)
        self.thread_ID = thread_ID
        self.name = name
        self.drone_name = drone_name
        self.q2 = q2
        self.location_json = location_json

    def run(self):
        with open(self.location_json) as f:
            data = json.load(f)
        coord = data['Vehicles'][self.drone_name]
        start_coord = (coord['X'], coord['Y'], coord['Z'])
        print("Starting thread " + self.drone_name)
        print(self.drone_name, start_coord)
        data_gatherer = DataGathering(vehicle_name=self.drone_name, vehicle_start_position=start_coord)
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
    location_json_file = "C:\\Users\jialv\OneDrive\Documentos\AirSim\settings.json"
    flight_info_folder = "Flight_info"
    existing_datasets = os.listdir(flight_info_folder)
    with open(location_json_file) as f:
        data = json.load(f)
    drones_available = list(data['Vehicles'].keys())

    # Creation of threads
    client_Queue = queue.Queue(1)
    number_drones_available = len(drones_available)
    it = 2
    for run in range(args.number_runs):
        threads = []
        for number in range(number_drones_available):
            thread_number = number_drones_available*run+number+1
            print("thread_number = ", thread_number)
            thread = MultiThreadDataGathering(thread_number, str(thread_number), drones_available[number], client_Queue,
                                              location_json_file)
            threads.append(thread)

        for number in range(number_drones_available):
            threads[number].start()

        for t in threads:
            t.join()
        client_Queue.get().reset(True)
        it = merge_flight_infos(existing_datasets, flight_info_folder, run, it)
