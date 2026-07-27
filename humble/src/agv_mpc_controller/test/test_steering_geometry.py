import math

import pytest

from agv_mpc_controller.steering_geometry import (
    front_two_axle_kinematics,
    front_two_axle_wheel_angles,
)


TREAD = 2.381384
AXLE_POSITIONS = (4.65, 3.0, -3.0, -4.65)
AXLE1_EQUIVALENT_WHEELBASE = 6.116611371515338
AXLE2_EQUIVALENT_WHEELBASE = 3.5527407651152627


def test_maximum_inner_outer_angles_and_fixed_rear_axles():
    angles = front_two_axle_wheel_angles(
        0.5229795759057247,
        0.45056049764376704,
        AXLE1_EQUIVALENT_WHEELBASE,
        AXLE2_EQUIVALENT_WHEELBASE,
        TREAD,
    )
    assert math.degrees(angles[0]) == pytest.approx(33.0)
    assert math.degrees(angles[1]) == pytest.approx(27.4)
    assert math.degrees(angles[2]) == pytest.approx(30.0)
    assert math.degrees(angles[3]) == pytest.approx(22.6)
    assert angles[4:] == (0.0, 0.0, 0.0, 0.0)


def test_prediction_is_straight_and_mirrors_turns():
    straight = front_two_axle_kinematics(
        3.0,
        0.0,
        0.0,
        AXLE_POSITIONS,
        TREAD,
        AXLE1_EQUIVALENT_WHEELBASE,
        AXLE2_EQUIVALENT_WHEELBASE,
    )
    assert straight == pytest.approx((0.0, 0.0))

    left = front_two_axle_kinematics(
        3.0,
        0.2,
        0.12,
        AXLE_POSITIONS,
        TREAD,
        AXLE1_EQUIVALENT_WHEELBASE,
        AXLE2_EQUIVALENT_WHEELBASE,
    )
    right = front_two_axle_kinematics(
        3.0,
        -0.2,
        -0.12,
        AXLE_POSITIONS,
        TREAD,
        AXLE1_EQUIVALENT_WHEELBASE,
        AXLE2_EQUIVALENT_WHEELBASE,
    )
    assert left[0] == pytest.approx(-right[0])
    assert left[1] == pytest.approx(-right[1])
    assert left[1] > 0.0
