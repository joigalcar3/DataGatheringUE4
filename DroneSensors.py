import airsim
import os
import time
import numpy as np
from icecream import ic
from DroneCamera import DroneCamera
from utils import transform_list_to_string, unwrapping_json
from user_input import load_user_input


class DroneSensors:
    """
    Class that serves as interface between the drone and all the sensors
    """
    # Approximate duration of a second in Unreal Engine 4. Computed by taking the average of 1000 real seconds with
    # the drone in stand-by.
    UE4_second = 993705198.592

    def __init__(self, client, sensors, cameras_info, sample_rates, folder='Sensor_data', vehicle_name=''):
        self.client = client
        self.folder = os.path.join(os.getcwd(), folder)
        self.flight_folder_location = None
        self.folder_name = None
        self.vehicle_name = vehicle_name
        self.sample_rates = sample_rates

        self.sensors = sensors
        self.number_sensors = len(sensors)

        self.cameras_info = cameras_info
        self.number_cameras = len(list(self.cameras_info.keys()))
        self.cameras = []

        self.data = {}
        self.headers = {}
        self.last_sample_time = {sensor: 0 for sensor in self.sensors}
        self.last_sample_time['camera'] = 0

    def initialize_signal_sensors(self):
        """
        Method which initialises all the sensors listed in self.sensors, except the camera
        :return:
        """
        for sensor in self.sensors:
            self.initialize_signal_sensor(sensor)

    def initialize_signal_sensor(self, sensor_type):
        """
        Method that initialises the dictionary that stores the data and saves the header that is going to be used when
        storing the data. It supports barometer, gps, magnetometer, imu
        :param sensor_type: type of the sensor
        :return:
        """
        # Obtain the name of the function to be run
        name_func = 'get' + sensor_type.capitalize() + 'Data'

        # Call the function
        output = getattr(self.client, name_func)(vehicle_name=self.vehicle_name)
        content = output.__dict__

        # Obtain the name of all the variables stored in the dictionary.
        headers = list(content.keys())
        header_names = unwrapping_json(headers, output, keys_values='keys')  # Extract the information from AirSim tree

        # Puts the names of all the variables together in a single string to start the Excel used for data storage
        headers_str = transform_list_to_string(header_names)
        self.headers[sensor_type] = headers_str
        self.data[sensor_type] = []

    def initialize_cameras(self):
        """
        Initialize the camera sensors
        :return:
        """
        for camera in self.cameras_info.keys():
            self.cameras.append(DroneCamera(camera, self.cameras_info[camera], self.client, self.flight_folder_location,
                                            vehicle_name=self.vehicle_name))

    def initialize_sensors(self):
        """
        Initialize all the sensors
        :return:
        """
        # Create the folder where the data of the flight will be saved
        try:
            self.flight_folder_location = os.path.join(self.folder,
                                                       time.strftime("%Y%m%d-%H%M%S") + "_" + self.vehicle_name[-1])
            self.folder_name = time.strftime("%Y%m%d-%H%M%S") + "_" + self.vehicle_name[-1]
        except:
            self.flight_folder_location = os.path.join(self.folder, time.strftime("%Y%m%d-%H%M%S"))
            self.folder_name = time.strftime("%Y%m%d-%H%M%S")
        os.mkdir(self.flight_folder_location)

        # Initialize all the sensors
        self.initialize_signal_sensors()
        self.initialize_cameras()

    def store_signals_sensors_data(self):
        """
        Store the data from the sensors, except the cameras. The data is still not dumped in the storage files.
        This prevents the constant calling of write commands to files in the directory.
        :return:
        """
        for sensor in self.sensors:
            self.store_signal_sensor_data(sensor)

    def store_signal_sensor_data(self, sensor_type):
        """
        Store the data from each of the sensors, except the cameras.
        :param sensor_type: the type of sensor, whether it is a gps, barometer, magnetometer or imu
        :return:
        """
        # Check whether data has tobe collected according to the sample rate of the sensor
        sample_rate, time_old, time_now = self.sample_rates[sensor_type], self.last_sample_time[sensor_type], \
                                          self.client.getMultirotorState().timestamp
        if (self.UE4_second / sample_rate + time_old) < time_now:
            self.last_sample_time[sensor_type] = time_now
            # Obtain the name of the AirSim method that has to be called
            name_func = "".join(['get' + sensor_type.capitalize() + 'Data'])
            output = getattr(self.client, name_func)(vehicle_name=self.vehicle_name)
            self.data[sensor_type].append(output)

    def store_camera_data(self):
        """
        Store the camera information
        :return:
        """
        sample_rate, time_old, time_now = self.sample_rates['camera'], self.last_sample_time['camera'], \
                                          self.client.getMultirotorState().timestamp
        if (self.UE4_second / sample_rate + time_old) < time_now:
            camera_requests = [camera.obtain_camera_image() for camera in self.cameras]
            responses = self.client.simGetImages(camera_requests, vehicle_name=self.vehicle_name)
            for i in range(self.number_cameras):
                self.cameras[i].store_camera_image(responses[i])

    def store_sensors_data(self):
        """
        Store the information from all the sensors
        :return:
        """
        self.store_signals_sensors_data()
        self.store_camera_data()

    def write_signal_sensor_to_file(self):
        """
        Write the stored information to their respective files for each of the sensors
        :return:
        """
        for sensor in self.data.keys():
            filename = sensor + ".csv"
            full_path = os.path.join(self.flight_folder_location, filename)
            output = self.data[sensor]
            unwrapped_data_lst = []
            for data_point in output:
                content = data_point.__dict__

                # Obtain the data stored by the tree structured returned by AirSim
                headers = list(content.keys())
                unwrapped_data = unwrapping_json(headers, data_point, keys_values='values', predecessor='', unwrapped_info=None)
                unwrapped_data_lst.append(unwrapped_data)
            data_points = np.array(unwrapped_data_lst)
            header = self.headers[sensor]
            np.savetxt(full_path, data_points, delimiter=',', header=header)

    def write_camera_to_file(self):
        """
        Write the stored images from the camera to their respective files
        :return:
        """
        for i in range(self.number_cameras):
            self.cameras[i].write_camera_to_file()

    def write_to_file(self):
        """
        Write to the file the information stored by all the sensors
        :return:
        """
        self.write_signal_sensor_to_file()
        self.write_camera_to_file()
        self.restart_sensors()

    def restart_sensors(self):
        """
        All the information about the sensors is restarted once the data has been written to disk.
        :return:
        """
        self.cameras = []
        self.flight_folder_location = None
        self.folder_name = None
        self.data = {}
        self.headers = {}
        self.last_sample_time = {sensor: 0 for sensor in self.sensors}
        self.last_sample_time['camera'] = 0

    def run(self):
        """
        Run the writing of data from the sensors at a sample rate of 10Hz for 1 second
        :return:
        """
        for i in range(10):
            self.store_sensors_data()
            time.sleep(0.1)
        self.write_to_file()


if __name__ == "__main__":
    # Input from the user
    args = load_user_input()

    # Object inputs
    client = airsim.MultirotorClient()
    sensors = DroneSensors(client, args.sensors_lst, args.cameras_info, args.sample_rates)
    sensors.run()
