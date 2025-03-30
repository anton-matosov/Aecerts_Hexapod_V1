import math
from bezier import Vector3, print_value
from hexapod_initializations import (
    a1,
    a2,
    a3,
    coxa1,
    coxa2,
    coxa3,
    coxa4,
    coxa5,
    coxa6,
    current_points,
    cycle_start_points,
    femur1,
    femur2,
    femur3,
    femur4,
    femur5,
    femur6,
    leg_length,
    rotate_sim_legs,
    servos_attached,
    tibia1,
    tibia2,
    tibia3,
    tibia4,
    tibia5,
    tibia6,
)
from hexapod_initializations import (
    attach_servos,
)
from globals import (
    constrain,
    g,
)
from nrf import hex_sensor_data


# AM - checked
# merge of 2 setters to make it pythonic
def set_cycle_start_points(leg=None):
    if leg is not None:
        g.cycle_start_points[leg] = g.current_points[leg]
    else:
        for i in range(6):
            g.cycle_start_points[i] = g.current_points[i]


# AM - checked
def rotate_to_angle(leg, target_rot):
    global servos_attached

    rotate_sim_legs(leg, target_rot)
    if not servos_attached:
        attach_servos()

    if leg == 0:
        coxa1.write_angle(target_rot.x)
        femur1.write_angle(target_rot.y)
        tibia1.write_angle(target_rot.z)
    elif leg == 1:
        coxa2.write_angle(target_rot.x)
        femur2.write_angle(target_rot.y)
        tibia2.write_angle(target_rot.z)
    elif leg == 2:
        coxa3.write_angle(target_rot.x)
        femur3.write_angle(target_rot.y)
        tibia3.write_angle(target_rot.z)
    elif leg == 3:
        coxa4.write_angle(target_rot.x)
        femur4.write_angle(target_rot.y)
        tibia4.write_angle(target_rot.z)
    elif leg == 4:
        coxa5.write_angle(target_rot.x)
        femur5.write_angle(target_rot.y)
        tibia5.write_angle(target_rot.z)
    elif leg == 5:
        coxa6.write_angle(target_rot.x)
        femur6.write_angle(target_rot.y)
        tibia6.write_angle(target_rot.z)


# AM - checked
def move_to_pos(leg, pos):
    global hex_sensor_data, servos_attached, current_points, target_rot

    hex_sensor_data.foot_positions[leg].x = int(pos.x)
    hex_sensor_data.foot_positions[leg].y = int(pos.y)

    if not servos_attached:
        attach_servos()

    current_points[leg] = pos

    dis = Vector3(0, 0, 0).distance_to(pos)
    if dis > leg_length:
        print_value('Point impossible to reach', pos, False)
        print_value('Distance', dis, True)
        return

    x = pos.x
    y = pos.y
    z = pos.z

    o1 = offsets[leg].x
    o2 = offsets[leg].y
    o3 = offsets[leg].z

    theta1 = math.atan2(y, x) * (180 / math.pi) + o1  # base angle
    l = math.sqrt(x * x + y * y)  # x and y extension
    l1 = l - a1
    h = math.sqrt(l1 * l1 + z * z)

    phi1 = math.acos(constrain((h**2 + a2**2 - a3**2) / (2 * h * a2), -1, 1))
    phi2 = math.atan2(z, l1)
    theta2 = (phi1 + phi2) * 180 / math.pi + o2
    phi3 = math.acos(constrain((a2**2 + a3**2 - h**2) / (2 * a2 * a3), -1, 1))
    theta3 = 180 - (phi3 * 180 / math.pi) + o3

    target_rot = Vector3(theta1, theta2, theta3)

    # TODO(AM): use rotateToAngle
    rotate_to_angle(leg, target_rot)
