import math


def ackermann_wheel_angles(axle_center_steer, equivalent_wheelbase, wheel_tread):
    """Return left/right wheel angles from an axle-center equivalent steer angle."""
    if (
        abs(axle_center_steer) < 1e-9
        or equivalent_wheelbase <= 1e-9
        or wheel_tread <= 1e-9
    ):
        return axle_center_steer, axle_center_steer

    center_radius = equivalent_wheelbase / math.tan(abs(axle_center_steer))
    half_tread = 0.5 * wheel_tread
    inner_angle = math.atan(equivalent_wheelbase / max(1e-6, center_radius - half_tread))
    outer_angle = math.atan(equivalent_wheelbase / (center_radius + half_tread))
    if axle_center_steer > 0.0:
        return inner_angle, outer_angle
    return -outer_angle, -inner_angle


def front_two_axle_wheel_angles(
    axle1_center_steer,
    axle2_center_steer,
    axle1_equivalent_wheelbase,
    axle2_equivalent_wheelbase,
    wheel_tread,
):
    axle1 = ackermann_wheel_angles(
        axle1_center_steer, axle1_equivalent_wheelbase, wheel_tread
    )
    axle2 = ackermann_wheel_angles(
        axle2_center_steer, axle2_equivalent_wheelbase, wheel_tread
    )
    return (*axle1, *axle2, 0.0, 0.0, 0.0, 0.0)


def fit_wheel_kinematics(longitudinal_velocity, axle_positions, wheel_tread, wheel_steers):
    """Least-squares planar no-slip fit over all eight physical wheel stations."""
    half_tread = 0.5 * wheel_tread
    rows = []
    for wheel_index, wheel_steer in enumerate(wheel_steers):
        axle_x = axle_positions[wheel_index // 2]
        wheel_y = half_tread if wheel_index % 2 == 0 else -half_tread
        steer_tangent = math.tan(wheel_steer)
        rows.append(
            (axle_x + wheel_y * steer_tangent, longitudinal_velocity * steer_tangent)
        )

    count = float(len(rows))
    sum_a1 = sum(row[0] for row in rows)
    sum_a1_a1 = sum(row[0] * row[0] for row in rows)
    sum_b = sum(row[1] for row in rows)
    sum_a1_b = sum(row[0] * row[1] for row in rows)
    denominator = count * sum_a1_a1 - sum_a1 * sum_a1
    if abs(denominator) <= 1e-9:
        return 0.0, 0.0
    lateral_velocity = (sum_b * sum_a1_a1 - sum_a1 * sum_a1_b) / denominator
    yaw_rate = (count * sum_a1_b - sum_a1 * sum_b) / denominator
    return lateral_velocity, yaw_rate


def front_two_axle_kinematics(
    longitudinal_velocity,
    axle1_center_steer,
    axle2_center_steer,
    axle_positions,
    wheel_tread,
    axle1_equivalent_wheelbase,
    axle2_equivalent_wheelbase,
):
    wheel_steers = front_two_axle_wheel_angles(
        axle1_center_steer,
        axle2_center_steer,
        axle1_equivalent_wheelbase,
        axle2_equivalent_wheelbase,
        wheel_tread,
    )
    return fit_wheel_kinematics(
        longitudinal_velocity, axle_positions, wheel_tread, wheel_steers
    )
