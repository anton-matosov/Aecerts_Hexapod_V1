import time
from bezier import Vector2, Vector3, get_point_on_bezier_curve, map_float
from globals import State, g
from hexapod_control import move_to_pos, set_cycle_start_points
from nrf import rc_control_data


# AM - checked
def slam_attack():
    set_cycle_start_points()
    g.current_state = State.SLAM_ATTACK
    g.slam_started = False

    attack_speed = map_float(rc_control_data.slider1, 0, 100, 20, 100)
    attack_speed = 25

    frames = attack_speed * 0.4
    for i in range(int(frames)):
        t = float(i) / frames
        move_to_pos(0, get_foot_placement_path_point(0, t))
        move_to_pos(1, get_foot_placement_path_point(1, t))
        move_to_pos(2, get_foot_placement_path_point(2, t))
        move_to_pos(3, get_foot_placement_path_point(3, t))
        move_to_pos(4, get_foot_placement_path_point(4, t))
        move_to_pos(5, get_foot_placement_path_point(5, t))

    set_cycle_start_points()

    frames = attack_speed * 1.2
    for i in range(int(frames)):
        t = float(i) / frames
        move_to_pos(0, get_leap_path_point(0, t))
        move_to_pos(1, get_leap_path_point(1, t))
        move_to_pos(4, get_leap_path_point(4, t))
        move_to_pos(5, get_leap_path_point(5, t))

        move_to_pos(2, get_slam_path_point(2, t))
        move_to_pos(3, get_slam_path_point(3, t))
        if t >= 0.5 and not g.slam_started:
            g.slam_started = True

    time.sleep(0.1)
    set_cycle_start_points()


# AM - checked
def get_foot_placement_path_point(leg, t):
    x_offset = 0
    y_offset = 0
    z_offset = 0

    if leg == 1:
        z_offset = -60
        y_offset = -50
        x_offset = -70
    if leg == 4:
        z_offset = -50
        y_offset = -60
        x_offset = -70

    if leg == 0:
        x_offset = 40
    if leg == 5:
        x_offset = 40

    x = g.cycle_start_points[leg].x + x_offset

    g.control_points[0] = g.cycle_start_points[leg]
    g.control_points[1] = Vector3(x, -50 * g.stride_multiplier[leg], -50 + z_offset).rotate(
        55 * g.rotation_multiplier[leg], Vector2(x, 0)
    )
    point = get_point_on_bezier_curve(g.control_points, 2, t)

    return point


# AM - checked
def get_leap_path_point(leg, t):
    x = g.cycle_start_points[leg].x
    start = g.cycle_start_points[leg]
    end = Vector3(
        x - 20, g.cycle_start_points[leg].y + (160 * g.stride_multiplier[leg]), -80
    ).rotate(55 * g.rotation_multiplier[leg], Vector2(x, 0))
    middle = ((start + end) * 0.5) + Vector3(0, 0, -300)

    if leg == 0 or leg == 5:
        middle.z += 180

    g.control_points[0] = start
    g.control_points[1] = middle
    g.control_points[2] = end
    point = get_point_on_bezier_curve(g.control_points, 3, t)
    return point


# AM - checked
def get_slam_path_point(leg, t):
    slam_percentage = 0.70
    land_percentage = 0.95

    # Leg Raise
    if t < slam_percentage:
        g.control_points[0] = g.cycle_start_points[leg]
        g.control_points[1] = Vector3(200, 0, 200).rotate(
            -40 * g.rotation_multiplier[leg], Vector2(0, 0)
        )
        g.control_points[2] = Vector3(0, 0, 300).rotate(
            -35 * g.rotation_multiplier[leg], Vector2(0, 0)
        )
        point = get_point_on_bezier_curve(
            g.control_points, 3, map_float(t, 0, slam_percentage, 0, 1)
        )
        return point

    # Leg Slam
    if slam_percentage <= t < land_percentage:
        g.control_points[0] = Vector3(0, 0, 300).rotate(
            -35 * g.rotation_multiplier[leg], Vector2(0, 0)
        )
        g.control_points[1] = Vector3(300, 0, 300).rotate(
            -35 * g.rotation_multiplier[leg], Vector2(0, 0)
        )
        g.control_points[2] = Vector3(325, 0, 50).rotate(
            -35 * g.rotation_multiplier[leg], Vector2(0, 0)
        )
        g.control_points[3] = Vector3(250, 0, 0).rotate(
            -35 * g.rotation_multiplier[leg], Vector2(0, 0)
        )
        point = get_point_on_bezier_curve(
            g.control_points, 4, map_float(t, slam_percentage, land_percentage, 0, 1)
        )
        return point

    if t >= land_percentage:
        return Vector3(250, 0, 0).rotate(-35 * g.rotation_multiplier[leg], Vector2(0, 0))
