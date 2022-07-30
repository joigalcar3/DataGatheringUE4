import warnings


def verify_user_input(parser):
    test_content = parser.parse_args()

    # Arguments related to the altitude selection
    if test_content.altitude_m >= 0:
        warnings.warn("The altitude has been fixed to a single value by the user_input: altitude_m >= 0")

    # Arguments related to the occupancy map generation
    if test_content.update_saved_vertices:
        warnings.warn(f"The saved vertices in {test_content.saved_vertices_filename} will be updated.")

    # Arguments related to the drone navigation
    if test_content.min_flight_distance_m <= 10:
        warnings.warn(f"The minimum flight distance is not larger than 10 m: {test_content.min_flight_distance_m}.")

    if test_content.min_flight_distance_m - test_content.max_flight_distance_m <= 5:
        warnings.warn(f"There is not enough flight distance for injecting a failure.")

    if test_content.start is not None:
        warnings.warn(f"The start position has been fixed by the user")

    if test_content.goal is not None:
        warnings.warn(f"The goal position has been fixed by the user")
