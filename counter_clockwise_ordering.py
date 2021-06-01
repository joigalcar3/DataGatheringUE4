import math


def counter_clockwise_ordering(points):
    """
    Function which rearranges a group of points of a grid clockwise. For that purpose, the centroid is found and they
    are ordered according to the angle they create by connecting them to the centroid.
    :param points: the list of point coordinates
    :return:
    """
    if type([]) != type(points):
        points = points.tolist()

    # compute centroid
    centroid = (sum([p[0] for p in points]) / len(points), sum([p[1] for p in points]) / len(points))

    # sort by polar angle
    points.sort(key=lambda p: math.atan2(p[1] - centroid[1], p[0] - centroid[0]))

    return points


if __name__ == "__main__":
    pp = [[26, 46],
          [23, 49],
          [23, 46],
          [26, 49],
          [23, 50],
          [26, 50]]
    output = counter_clockwise_ordering(pp)
    print(output)
