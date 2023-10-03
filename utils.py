#!/usr/bin/env python
"""
Provides the function that help in the transformation of the UE4 point cloud into an occupancy map and the processing of
data pulled from the simulation.
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Stable"

# Imports
import csv
import os
import pickle
from math import *
from matplotlib.pyplot import *
from scipy.spatial import Delaunay


def compute_distance_points(pointA, pointB):
    """
    Compute the distance within 2 points in the x-y plane
    :param pointA: start point
    :param pointB: target point
    :return: distance between both provided points
    """
    distance = np.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)
    return distance


def counter_clockwise_ordering(points):
    """
    Function which rearranges a group of points of a grid clockwise. For that purpose, the centroid is found and they
    are ordered according to the angle they create by connecting them to the centroid.
    :param points: the list of point coordinates
    :return: the list of the points sorted clockwise
    """
    if type([]) != type(points):
        points = points.tolist()

    # Compute centroid
    centroid = (sum([p[0] for p in points]) / len(points), sum([p[1] for p in points]) / len(points))

    # Sort by polar angle
    points.sort(key=lambda p: atan2(p[1] - centroid[1], p[0] - centroid[0]))

    return points


def transform_list_to_string(list_to_transform):
    """
    Simple method to transform a list of strings to a single string with each of the list elements separated by comma
    :param list_to_transform: list of strings
    :return: single string with all the strings from the list
    """
    headers_str = ''
    for i in range(len(list_to_transform)):
        if i != 0:
            header = ',' + list_to_transform[i]
        else:
            header = list_to_transform[i]
        headers_str += header

    return headers_str


def unwrapping_json(branch, output, keys_values, predecessor='', unwrapped_info=None):
    """
    Recursive method that unwraps all the information stored by the AirSim tree data structure to a list. This is done
    such that the information can be later stored in .csv format
    :param branch: list of the keys corresponding to the variables stored in the next level deep of the AirSim tree, and
    that should be 'investigated' in the next recursion
    :param output: branch within the AirSim data structure that is next to be unfolded
    :param keys_values: whether the keys or the values are to be stored
    :param predecessor: for a header (key) n levels deep in the data structure, it is saved as follows:
    first_level.second_level.____.nth_level. In order to achieve this notation, the function is passed the state of the
    naming up until the previous level. Therefore, for the third level, the predecessor would be:
    first_level.second_level.
    :param unwrapped_info: the list with the information that will be returned by this method
    :return: list containing the unwrapped information
    """
    # Initialise the list to stored the output
    if unwrapped_info is None:
        unwrapped_info = []

    # For each of the keys in the current branch of the tree
    for element in branch:
        try:
            # The list of the keys that should be 'investigated' in the next recursion
            branch_recursive = list(getattr(output, element).__dict__.keys())

            # The state of the tree in the current branch
            output_recursive = getattr(output, element)

            # Current naming of the headers
            predecessor_recursive = predecessor + element + '.'

            unwrapped_info_deeper = unwrapping_json(branch_recursive, output_recursive, keys_values,
                                                    predecessor_recursive)

            # Store the output information provided by deeper layers of the tree
            unwrapped_info += unwrapped_info_deeper
        except:
            if keys_values == 'keys':
                # In the case that header information should be stored
                predecessor_recursive = predecessor + element
                unwrapped_info.append(predecessor_recursive)
            elif keys_values == 'values':
                # In the case that the data should be stored
                data = getattr(output, element)

                # Translate booleans to integers
                if type(data) == bool:
                    data = int(data)

                # Remove empty lists from the output
                elif type(data) == list:
                    data = -1
                unwrapped_info.append(data)
    return unwrapped_info


def merge_flight_infos(original_datasets, fi_folder, run, it):
    """
    Function that merges multiple flight information files into a single one. This is useful when multiple data
    gathering runs were executed, each generating a different flight info file. However, they need to combine in order
    to generate a single dataset
    :param original_datasets: the name of the flight info files that already exist
    :param fi_folder: the folder where the flight info files are located
    :param run: the number of the current data gathering run
    :param it: the number of flights that have been processed, it is used as the index of the final flight information
    file
    :return: the current state of the "it" variable
    """
    # Flight info files. Checking whether any files were added to the directory
    files = os.listdir(fi_folder)
    new_files = np.setdiff1d(files, original_datasets)
    original_file = new_files[0]

    # For the first run
    if run == 0:
        with open(os.path.join(fi_folder, original_file)) as f_start:
            csv_reader = csv.reader(f_start, delimiter=',')
            counter = -1
            rows = []
            for row in csv_reader:
                counter += 1
                rows.append(row)
        if counter != 1:
            with open(os.path.join(fi_folder, original_file), 'w', encoding='UTF8', newline='') as f_start:
                csv_writer = csv.writer(f_start, delimiter=',')
                csv_writer.writerow(rows[0])
                it = 1
                for i in range(1, len(rows)):
                    row = rows[i]
                    row[0] = str(it)
                    csv_writer.writerow(row)
                    it += 1
    else:
        it = it

    # Add the rest of the new files (those corresponding to the data collected by the rest of the drones except the
    # first one)
    drone_created_files = new_files[1:]
    with open(os.path.join(fi_folder, original_file), 'a', encoding='UTF8', newline='') as f_original:
        original_writer = csv.writer(f_original, delimiter=',')
        for file in drone_created_files:
            with open(os.path.join(fi_folder, file)) as f_run:
                csv_reader = csv.reader(f_run, delimiter=',')
                header = True
                for row in csv_reader:
                    if header:
                        header = False
                    else:
                        row[0] = str(it)
                        original_writer.writerow(row)  # add the row to the main file
                        it += 1

            # Once the contents from the file have been translated to the main file, the new file is deleted
            os.remove(os.path.join(fi_folder, file))
    return it


def obtain_outer_edge(points, alpha, only_outer=True):
    """
    Compute the alpha shape (concave hull) of a set of points. The higher the alpha, the larger the polygon considered.
    :param points: np.array of shape (n,2) points
    :param alpha: alpha value
    :param only_outer: boolean value to specify if we keep only the outer border
    or also inner edges
    :return: set of (i,j) pairs representing edges of the alpha-shape. (i,j) are
    the indices in the points array. Also, a list of all the points that shape the convex hull.
    """
    assert points.shape[0] > 3, "Need at least four points"

    def add_edge(edges, i, j):
        """
        Add an edge between the i-th and j-th points, if not in the list already
        :param edges: tuples of points
        :param i: first point
        :param j: second point
        :return: None
        """
        if (i, j) in edges or (j, i) in edges:
            # already added
            assert (j, i) in edges, "Can't go twice over same directed edge right?"
            if only_outer:
                # if both neighboring triangles are in shape, it's not a boundary edge
                edges.remove((j, i))
            return
        edges.add((i, j))

    # Obtain the triangulation from Delaunay and initiate a set of edges
    tri = Delaunay(points)
    edges = set()

    # Loop over triangles:
    # ia, ib, ic = indices of corner points of the triangle
    for ia, ib, ic in tri.vertices:
        pa = points[ia]
        pb = points[ib]
        pc = points[ic]

        # Computing radius of triangle circumcircle
        # www.mathalino.com/reviewer/derivation-of-formulas/derivation-of-formula-for-radius-of-circumcircle
        a = np.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
        b = np.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
        c = np.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)
        s = (a + b + c) / 2.0
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        circum_r = a * b * c / (4.0 * area)

        # Only add triangles that fit in circles whose radius is smaller than circum_r
        if circum_r < alpha:
            add_edge(edges, ia, ib)
            add_edge(edges, ib, ic)
            add_edge(edges, ic, ia)

    # The convex hull is defined by the points that still remain after adding the edges of all the triangles to the
    # edges set
    point_indices = list(set([point for edge in edges for point in edge]))
    output_points = points[point_indices]
    return edges, output_points


def depickle(directory, filename):
    """
    Function to retrieve the information that has been pickled
    :param directory: location where the file has been stored
    :param filename: name of the file
    :return: contents of the file
    """
    filename = os.path.join(directory, filename)
    with open(filename, 'rb') as f:
        contents = pickle.load(f)
    return contents


if __name__ == "__main__":
    # Simple implementation which shows the functionality of function that obtains the outer edges of a cloud of points
    # in 2D
    # Constructing the input point data
    np.random.seed(0)
    x = 3.0 * np.random.rand(2000)
    y = 2.0 * np.random.rand(2000) - 1.0
    inside = ((x ** 2 + y ** 2 > 1.0) & ((x - 3) ** 2 + y ** 2 > 1.0))
    points = np.vstack([x[inside], y[inside]]).T

    # Computing the alpha shape
    edges, _ = obtain_outer_edge(points, alpha=0.5, only_outer=True)

    # Plotting the output
    figure()
    axis('equal')
    plot(points[:, 0], points[:, 1], '.')
    for i, j in edges:
        plot(points[[i, j], 0], points[[i, j], 1])
    show()
