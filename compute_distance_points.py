import numpy as np

def compute_distance_points(pointA, pointB):
    """
    Compute the distance within 2 points in the x-y plane
    :param pointA: start point
    :param pointB: target point
    :return: distance between both provided points
    """
    distance = np.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)
    return distance