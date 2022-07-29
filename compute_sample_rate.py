from user_input import load_user_input
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

# Set up the plotting backend
mpl.use('TKAgg')

# User input
user_input = load_user_input()
flight_data_number = 27

# Creating necessary storage folders
storage_folder = os.path.join(os.getcwd(), "Sampling_rate_analysis")
isExist = os.path.exists(storage_folder)
if not isExist:
    os.makedirs(storage_folder)

# Retrieving the folders to analyse with sensor information
sensors_remote_storage_location = "E:\\AirSim_project"
flight_info_remote_storage_location = "E:\\AirSim_project"
# sensors_remote_storage_location = user_input.sensors_remote_storage_location
# flight_info_remote_storage_location = user_input.flight_info_remote_storage_location
flight_info_folder = os.path.join(flight_info_remote_storage_location, "Flight_info")
flight_info_files = os.listdir(flight_info_folder)
flight_data_file = flight_info_files[[f"_{flight_data_number}_" in file for file in flight_info_files].index(True)]
flight_info = os.path.join(flight_info_remote_storage_location, "Flight_info", flight_data_file)
flight_data = pd.read_csv(flight_info)
folders_to_analyze = flight_data["Sensor_folder"].tolist()
sensor_folder = os.path.join(sensors_remote_storage_location, "Sensor_data")
number_folders = len(folders_to_analyze)

# Compute sampling rates
fps_lst = np.zeros(number_folders)
imu_lst = np.zeros(number_folders)
counter = 0
for folder in folders_to_analyze:
    # Compute image fps
    images_names = os.listdir(os.path.join(sensor_folder, folder, "front"))
    images_names.sort()
    if "0.png" in images_names:
        images_names.remove("0.png")
    first_image = int(images_names[0][:-4])
    last_image = int(images_names[-2][:-4])
    number_images = len(images_names)-1
    fps = number_images/((last_image-first_image)/1e9)
    fps_lst[counter] = fps

    # Compute the IMU sampling rate
    imu_file = os.path.join(sensor_folder, folder, "imu.csv")
    imu_info = pd.read_csv(imu_file)
    timestamps = imu_info['# timestamps']
    number_entries = len(timestamps)
    first_entry = timestamps[0]
    last_entry = timestamps[number_entries-1]
    imu_sampling_rate = number_entries/((last_entry-first_entry)/1e9)
    imu_lst[counter] = imu_sampling_rate
    counter += 1
    print(f"{folder} got a camera sampling rate of {fps} fps and an IMU sampling rate of {imu_sampling_rate} Hz.")
print(f"Mean camera fps: {np.mean(fps_lst)}")
print(f"Standard deviation camera fps: {np.std(fps_lst)}")
print(f"Mean IMU frequency: {np.mean(imu_lst)}")
print(f"Standard deviation IMU frequency: {np.std(imu_lst)}")

# Obtaining the image size
image_size_file = os.path.join(sensor_folder, folder, "front", "metadata.csv")
image_size_file_data = pd.read_csv(image_size_file)
heigth = int(image_size_file_data["height"][0])
width = int(image_size_file_data["width"][0])

# Obtaining the clockspeed
clockspeed = flight_data["ClockSpeed"][0]

# Plot the collected fps values
plt.figure(1)
plt.boxplot(fps_lst, meanline=True)
x_jitter = np.random.normal(1, 0.04, size=len(fps_lst))
plt.scatter(x_jitter, fps_lst)
plt.ylabel("Sample rate [fps]")
plt.title(f"Camera sampling rate. Image size: {heigth}x{width}. Clockspeed: {clockspeed}.")
plt.grid(True)
plt.savefig(os.path.join(storage_folder, f"{flight_data_file[:-4]}_camera.png"))

# Plot the collected imu sampling rate values
plt.figure(2)
plt.boxplot(imu_lst, meanline=True)
x_jitter = np.random.normal(1, 0.04, size=len(imu_lst))
plt.scatter(x_jitter, imu_lst)
plt.ylabel("Sample rate [Hz]")
plt.title(f"IMU sampling rate. Image size: {heigth}x{width}. Clockspeed: {clockspeed}.")
plt.grid(True)
plt.savefig(os.path.join(storage_folder, f"{flight_data_file[:-4]}_imu.png"))
plt.show()

