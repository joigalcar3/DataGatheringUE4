

# User input
if __name__ == '__main__':
    from Occupancy_grid.user_input import load_user_input
    from Occupancy_grid.DroneFlight import DroneFlight
    args = load_user_input()

    # Fly drone
    drone_flight = DroneFlight(args.altitude_m, args.altitude_range_m, args.cell_size_m,
                               args.ue4_airsim_conversion_units, args.robot_radius, args.sensors_lst, args.cameras_info,
                               args.sample_rate, min_flight_distance_m=args.min_flight_distance_m,
                               saved_vertices_filename=args.saved_vertices_filename,
                               update_saved_vertices=args.update_saved_vertices, plot2D=args.plot2D, plot3D=args.plot3D)
    drone_flight.run(navigation_type=args.navigation_type, start_point=args.start, goal_point=args.goal)