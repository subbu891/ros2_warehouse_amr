from amr_validation.traffic import robot_blocks_lane

FORKLIFT = (13.0, 4.0)


def blocks(robot, direction=1.0):
    return robot_blocks_lane(FORKLIFT, robot, direction, half_length=0.4, lane_half_width=0.95, brake_distance=1.0)


def test_robot_ahead_in_lane_blocks():
    assert blocks((13.0, 5.0))
    assert blocks((12.2, 5.2))


def test_robot_behind_or_far_ahead_does_not_block():
    assert not blocks((13.0, 3.0))
    assert not blocks((13.0, 6.0))


# A robot waiting outside the lane must not stop the forklift, otherwise both wait forever
def test_robot_waiting_beside_the_lane_does_not_block():
    assert not blocks((11.9, 5.0))
    assert not blocks((14.1, 5.0))


def test_direction_matters():
    assert blocks((13.0, 3.2), direction=-1.0)
    assert not blocks((13.0, 5.0), direction=-1.0)
