import threading
import queue
from Occupancy_grid.DataGathering import DataGathering
from Occupancy_grid.user_input import load_user_input
import time
import airsim


class MultiThreadDataGathering(threading.Thread):
    def __init__(self, thread_ID, name, drone_name, q2):
        threading.Thread.__init__(self)
        self.thread_ID = thread_ID
        self.name = name
        self.drone_name = drone_name
        self.q2 = q2

    def run(self):
        print("Starting thread " + self.drone_name)
        data_gatherer = DataGathering(self.drone_name)
        data_gatherer.drone_flight.run(min_h=data_gatherer.flight_altitudes[0], max_h=data_gatherer.flight_altitudes[1])
        if self.q2.empty():
            self.q2.put(data_gatherer.drone_flight.client)


if __name__ == "__main__":
    args = load_user_input()
    drones_available = ["Drone1", "Drone2", "Drone3", "Drone4"]

    client_Queue = queue.Queue(1)
    number_drones_available = len(drones_available)
    for run in range(args.number_runs):
        threads = []
        for number in range(number_drones_available):
            thread_number = number_drones_available*run+number+1
            print("thread_number = ", thread_number)
            thread = MultiThreadDataGathering(thread_number, str(thread_number), drones_available[number], client_Queue)
            threads.append(thread)

        for number in range(number_drones_available):
            threads[number].start()

        for t in threads:
            t.join()
        client_Queue.get().reset()



