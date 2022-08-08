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
# flight_data_numbers = list(range(34, 40))
flight_data_numbers = [41]
n_flight_data_numbers = len(flight_data_numbers)

# Creating necessary storage folders
storage_folder = os.path.join(os.getcwd(), "Sampling_rate_analysis")
isExist = os.path.exists(storage_folder)
if not isExist:
    os.makedirs(storage_folder)

# Retrieving the folders to analyse with sensor information
dataset_counter = 0
x_labels = []
fps_flights_lst = []
imu_flights_lst = []
for flight_data_number in flight_data_numbers:
    sensors_remote_storage_location = user_input.sensors_remote_storage_location
    flight_info_remote_storage_location = user_input.flight_info_remote_storage_location
    # sensors_remote_storage_location = user_input.sensors_remote_storage_location
    # flight_info_remote_storage_location = user_input.flight_info_remote_storage_location
    flight_info_folder = os.path.join(flight_info_remote_storage_location, "Flight_info")
    flight_info_files = os.listdir(flight_info_folder)
    flight_data_file = flight_info_files[[f"_{flight_data_number}_" in file for file in flight_info_files].index(True)]
    flight_info = os.path.join(flight_info_remote_storage_location, "Flight_info", flight_data_file)
    flight_data = pd.read_csv(flight_info)
    if not len(flight_data):
        continue
    folders_to_analyze = flight_data["Sensor_folder"].tolist()
    sensor_folder = os.path.join(sensors_remote_storage_location, "Sensor_data")
    number_folders = len(folders_to_analyze)

    # Compute sampling rates
    fps_lst = []
    imu_lst = []
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
        fps_lst.append(fps)

        # Compute the IMU sampling rate
        imu_file = os.path.join(sensor_folder, folder, "imu.csv")
        imu_info = pd.read_csv(imu_file)
        timestamps = imu_info['# timestamps']
        number_entries = len(timestamps)
        first_entry = timestamps[0]
        last_entry = timestamps[number_entries-1]
        imu_sampling_rate = number_entries/((last_entry-first_entry)/1e9)
        imu_lst.append(imu_sampling_rate)
        counter += 1
        print(f"{folder} got a camera sampling rate of {fps} fps and an IMU sampling rate of {imu_sampling_rate} Hz.")
    print(f"Mean camera fps: {np.mean(fps_lst)}")
    print(f"Standard deviation camera fps: {np.std(fps_lst)}")
    print(f"Mean IMU frequency: {np.mean(imu_lst)}")
    print(f"Standard deviation IMU frequency: {np.std(imu_lst)}")

    # Obtaining the image size
    image_size_file = os.path.join(sensor_folder, folders_to_analyze[0], "front", "metadata.csv")
    image_size_file_data = pd.read_csv(image_size_file)
    heigth = int(image_size_file_data["height"][0])
    width = int(image_size_file_data["width"][0])

    # Obtaining the clockspeed
    clockspeed = flight_data["ClockSpeed"][0]

    # Sensor desired sampling rate
    camera_fps = -1
    imu_frequency = -1
    if "Camera_fps" in flight_data:
        camera_fps = flight_data["Camera_fps"][0]
        imu_frequency = flight_data["IMU_frequency"][0]

    # Creating the correct xlabel according to the run information
    x_label = f"{flight_data_number}: {clockspeed}, {heigth}x{width}, C{camera_fps}, I{imu_frequency}"
    x_labels.append(x_label)

    dataset_counter += 1
    fps_flights_lst.append(fps_lst)
    imu_flights_lst.append(imu_lst)

# Plot the collected fps values
plt.figure(1)
ax1 = plt.gca()
plt.boxplot(fps_flights_lst, meanline=True)
counter = 1
for fps_lst in fps_flights_lst:
    x_jitter = np.random.normal(counter, 0.04, size=len(fps_lst))
    plt.scatter(x_jitter, fps_lst)
    counter += 1
plt.ticklabel_format(useOffset=False, axis='y')
ax1.set_xticklabels(x_labels)
plt.ylabel("Sample rate [fps]")
plt.title(f"Camera sampling rate.")
plt.grid(True)
plt.savefig(os.path.join(storage_folder, f"{flight_data_file[:-4]}_camera.png"))

# Plot the collected imu sampling rate values
plt.figure(2)
ax2 = plt.gca()
plt.boxplot(imu_flights_lst, meanline=True)
counter = 1
for imu_lst in imu_flights_lst:
    x_jitter = np.random.normal(counter, 0.04, size=len(imu_lst))
    plt.scatter(x_jitter, imu_lst)
    counter += 1
plt.ticklabel_format(useOffset=False, axis='y')
ax2.set_xticklabels(x_labels)
plt.ylabel("Sample rate [Hz]")
plt.title(f"IMU sampling rate.")
plt.grid(True)
plt.savefig(os.path.join(storage_folder, f"{flight_data_file[:-4]}_imu.png"))
plt.show()

