def robot_blocks_lane(forklift_xy, robot_xy, direction: float, half_length: float,
                      lane_half_width: float, brake_distance: float) -> bool:
    # The robot blocks the lane when it is inside the lane corridor and within brake_distance ahead
    ahead = (robot_xy[1] - forklift_xy[1]) * direction
    in_lane = abs(robot_xy[0] - forklift_xy[0]) < lane_half_width
    return in_lane and 0.0 < ahead < half_length + brake_distance
