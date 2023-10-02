#!/usr/bin/env python
"""
Provides the DroneCamera class that provides all the functions desired from an AirSim camera object, such as the
capturing of images and their structured storage for dataset build-up.
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Stable"

# Import
import airsim
import os
import numpy as np
import time
import cv2
from utils import transform_list_to_string, unwrapping_json


class DroneCamera:
    """
    Class that serves as interface for the DroneSensors class for the camera sensor
    """
    def __init__(self, alias, characteristics, client, flight_folder, vehicle_name=''):
        """
        Initializes the drone camera class
        :param alias: name given to the camera
        :param characteristics: characteristics of the camera
        :param client: AirSim client object
        :param flight_folder: flight path to the flight information folder
        :param vehicle_name: name of the vehicle
        """
        self.alias = alias  # Name give to the camera
        self.camera_name = characteristics["camera_name"]  # Name of the camera in AirSim
        self.image_type = characteristics["image_type"]    # What type of image do we want to obtain: vision, depth, seg
        characteristics_keys = list(characteristics.keys())
        self.vehicle_name = vehicle_name

        # Whether the image should be saved with floats instead of integers
        if "pixels_as_float" in characteristics_keys:
            self.pixels_as_float = characteristics["pixels_as_float"]
        else:
            self.pixels_as_float = False

        # Whether the image should be compressed
        if "compress" in characteristics_keys:
            self.compress = characteristics["compress"]
        else:
            self.compress = True

        # Create the folder whether the camera information can be stored
        self.client = client
        self.flight_folder = flight_folder
        self.camera_folder = os.path.join(self.flight_folder, alias)
        os.mkdir(self.camera_folder)

        self.stored_responses = {}
        self.width = None
        self.height = None

        self.store_camera_metadata()

    def store_camera_metadata(self):
        """
        Method to store the information from the camera prior to the flight in an Excel
        :return: None
        """
        camera_name = ''
        trial_counter = 0
        while camera_name == '':
            # In the case that the function is run faster than the camera is instantiated in UE4
            if trial_counter > 2:
                raise Exception("The camera metadata has an unknown camera name")
            elif trial_counter != 0:
                time.sleep(0.5)
            response = self.client.simGetImages([self.obtain_camera_image()], vehicle_name=self.vehicle_name)[0]
            camera_name = response.camera_name
            trial_counter += 1

        # Dimensions of the images taken by the camera
        self.width = response.width
        self.height = response.height

        # List the keys whose value are mutable and should not be stored
        keys_remove = ['image_data_uint8', 'image_data_float', 'message', 'time_stamp']
        header_lst = list(response.__dict__.keys())
        for i in keys_remove:
            header_lst.remove(i)

        # Store the header and data of the camera after unwrapping the AirSim tree data structure
        headers = unwrapping_json(header_lst, response, 'keys')
        headers_str = transform_list_to_string(headers)
        data_points = np.array([unwrapping_json(header_lst, response, 'values')], dtype=float)

        full_path = os.path.join(self.camera_folder, 'metadata.csv')
        np.savetxt(full_path, data_points, delimiter=',', header=headers_str)

    def store_camera_image(self, response):
        """
        Method that stores the data gathered by the camera
        :param response: the camera information received from AirSim for the present camera at a point in time after
        calling the client.simGetImages() method. The DroneCamera class does not perform the image call since it is
        better to call the images of all cameras simultaneously. In that way, there is a single call (computationally
        efficient) and we make sure that all images are taken at the same moment in time.
        :return: None
        """
        timestamp = response.time_stamp
        if self.pixels_as_float:
            self.stored_responses[timestamp] = response
        else:
            image = response.image_data_uint8
            self.stored_responses[timestamp] = image

    def write_camera_to_file(self):
        """
        Write the information stored by the camera objects to their respective files. A different approach is taken
        depending on whether the images should be stored as floats or whether they should be compressed.
        :return: None
        """
        for timestamp in self.stored_responses.keys():
            filename = os.path.join(self.camera_folder, str(timestamp))
            if self.pixels_as_float:  # store the pixels as floats
                print("Type %d, size %d" % (self.image_type, len(self.stored_responses[timestamp])))
                airsim.write_pfm(os.path.normpath(filename + '.pfm'),
                                 airsim.get_pfm_array(self.stored_responses[timestamp]))
            elif self.compress:  # png format
                print("Type %d, size %d" % (self.image_type, len(self.stored_responses[timestamp])))
                airsim.write_file(os.path.normpath(filename + '.png'), self.stored_responses[timestamp])
            else:  # uncompressed array
                print("Type %d, size %d" % (self.image_type, len(self.stored_responses[timestamp])))
                img1d = np.fromstring(self.stored_responses[timestamp], dtype=np.uint8)  # get numpy array
                img_rgb = img1d.reshape(self.height, self.width, 3)  # reshape array to 4 channel image array H X W X 3
                cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb)  # write to png

    def obtain_camera_image(self):
        """
        Method which calls an image only from the present camera object.
        :return:
        """
        return airsim.ImageRequest(self.camera_name, self.image_type, self.pixels_as_float, self.compress)


if __name__ == "__main__":
    # Simple use of the DroneCamera class
    client = airsim.MultirotorClient()  # AirSim client object
    folder = 'Sensor_data'  # folder name where the sensor data is stored
    folder = os.path.join(os.getcwd(), folder)
    flight_folder = os.path.join(folder, time.strftime("%Y%m%d-%H%M%S"))  # folder name where the camera data is stored
    os.mkdir(flight_folder)
    alias = 'front'  # alias of the camera
    cameras_info = {"camera_name": "0", "image_type": 0}  # camera characteristics
    camera = DroneCamera(alias, cameras_info, client, flight_folder)  # camera object
