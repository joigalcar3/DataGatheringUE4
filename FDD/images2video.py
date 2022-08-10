import cv2
import numpy as np
import glob

# File and path naming
data_name = "20220809-213754_1"
folder_name_images = f"F:/AirSim_project/Sensor_data/{data_name}/front"
folder_name_video = "C:\\Users\\jdealvearcarde\\AirSim\\multirotor\\DataGatheringUE4\\FDD\\Videos"
video_name = f"{data_name}.avi"

# Extract images and image size
img_array = []
for filename in glob.glob(f'{folder_name_images}/*.jpg'):
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width, height)
    img_array.append(img)


out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'DIVX'), 30, size)
for i in range(len(img_array)):
    out.write(img_array[i])
out.release()
