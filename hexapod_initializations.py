from bezier import Vector3
from globals import g
from models import LegModel
from point import Point3D

# Pin definitions
# Leg 0
coxa1_pin = 'left_back_coxa'  # 22
femur1_pin = 'left_back_femur'  # 23
tibia1_pin = 'left_back_tibia'  # 24

# Leg 1
coxa2_pin = 'left_middle_coxa'  # 25
femur2_pin = 'left_middle_femur'  # 26
tibia2_pin = 'left_middle_tibia'  # 27

# Leg 2
coxa3_pin = 'left_front_coxa'  # 28
femur3_pin = 'left_front_femur'  # 29
tibia3_pin = 'left_front_tibia'  # 30

# Leg 3
coxa4_pin = 'right_front_coxa'  # 31
femur4_pin = 'right_front_femur'  # 32
tibia4_pin = 'right_front_tibia'  # 33

# Leg 4
coxa5_pin = 'right_middle_coxa'  # 34
femur5_pin = 'right_middle_femur'  # 35
tibia5_pin = 'right_middle_tibia'  # 36

# Leg 5
coxa6_pin = 'right_back_coxa'  # 37
femur6_pin = 'right_back_femur'  # 38
tibia6_pin = 'right_back_tibia'  # 39

sim_legs: list[LegModel] = None


def setup_sim_legs(hexapod):
    global sim_legs
    sim_legs = [
        hexapod.left_back,
        hexapod.left_middle,
        hexapod.left_front,
        hexapod.right_front,
        hexapod.right_middle,
        hexapod.right_back,
    ]


def rotate_sim_leg(leg, target_rot):
    return

    if sim_legs is None:
        return

    sim_leg = sim_legs[leg]
    flip = sim_leg.label.startswith('right')
    sim_leg.forward_kinematics(-target_rot.x if flip else target_rot.x, -target_rot.y, target_rot.z)

    # foot = sim_legs[leg].tibia_end
    # foot_local = sim_legs[leg].to_local(foot)
    # print(f'rotaplt.sleepted leg: {sim_legs[leg].label=}, {target_rot=} {foot=} {foot_local=}')


def move_sim_leg_to_pos(leg, target_pos):
    # return
    if sim_legs is None:
        return

    sim_leg: LegModel = sim_legs[leg]
    flip = sim_leg.label.startswith('right')
    x = target_pos.x
    y = -target_pos.y if flip else target_pos.y
    # y = target_pos.y
    z = target_pos.z
    reached_target, alpha, beta, gamma = sim_leg.inverse_kinematics_local(Point3D([x, y, z]))

    sim_leg.forward_kinematics(alpha, beta, gamma)
    # print(f'moved leg: {sim_leg.label=}, {target_pos=} {alpha=} {beta=} {gamma=}')


# Hand crafted stub for Servo class
class Servo:
    def attach(self, joint_name, min_pulse, max_pulse):
        self.joint_name = joint_name
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse

    def detach(self):
        # Placeholder for detaching the servo
        pass

    def write_angle(self, angle):
        # Placeholder for writing angle in degrees
        ms = self.__angle_to_microseconds(angle)
        self.write_microseconds(ms)
        # print(f'{self.joint_name} {angle=} {ms=}')

    def write_microseconds(self, microseconds):
        # Placeholder for writing pulse width in microseconds
        pass

    def __angle_to_microseconds(self, angle):
        val = self.min_pulse + (((self.max_pulse - self.min_pulse) / 180.0) * angle)
        return int(val)

    def __microseconds_to_angle(self, microseconds):
        angle = ((microseconds - self.min_pulse) / (self.max_pulse - self.min_pulse)) * 180.0
        return angle


# AM - checked
# Individual servo objects
coxa1 = Servo()
femur1 = Servo()
tibia1 = Servo()

coxa2 = Servo()
femur2 = Servo()
tibia2 = Servo()

coxa3 = Servo()
femur3 = Servo()
tibia3 = Servo()

coxa4 = Servo()
femur4 = Servo()
tibia4 = Servo()

coxa5 = Servo()
femur5 = Servo()
tibia5 = Servo()

coxa6 = Servo()
femur6 = Servo()
tibia6 = Servo()


# AM - checked
# Leg dimensions
a1 = 46.0  # Coxa Length
a2 = 108.0  # Femur Length
a3 = 200.0  # Tibia Length
leg_length = a1 + a2 + a3


# AM - checked
# Position tracking
current_points = [Vector3() for _ in range(6)]
cycle_start_points = [Vector3() for _ in range(6)]


# AM - checked
current_rot = Vector3(180, 0, 180)
target_rot = Vector3(180, 0, 180)


# AM - checked
stride_multiplier = [1, 1, 1, -1, -1, -1]
rotation_multiplier = [-1, 0, 1, -1, 0, 1]


# AM - checked
control_points = [Vector3() for _ in range(10)]
rotate_control_points = [Vector3() for _ in range(10)]
attack_control_points = [Vector3() for _ in range(10)]


# AM - checked
def attach_servos():
    """Attach all servos to their pins with specified pulse width range"""
    try:
        coxa1.attach(coxa1_pin, 500, 2500)
        femur1.attach(femur1_pin, 500, 2500)
        tibia1.attach(tibia1_pin, 500, 2500)

        coxa2.attach(coxa2_pin, 500, 2500)
        femur2.attach(femur2_pin, 500, 2500)
        tibia2.attach(tibia2_pin, 500, 2500)

        coxa3.attach(coxa3_pin, 500, 2500)
        femur3.attach(femur3_pin, 500, 2500)
        tibia3.attach(tibia3_pin, 500, 2500)

        coxa4.attach(coxa4_pin, 500, 2500)
        femur4.attach(femur4_pin, 500, 2500)
        tibia4.attach(tibia4_pin, 500, 2500)

        coxa5.attach(coxa5_pin, 500, 2500)
        femur5.attach(femur5_pin, 500, 2500)
        tibia5.attach(tibia5_pin, 500, 2500)

        coxa6.attach(coxa6_pin, 500, 2500)
        femur6.attach(femur6_pin, 500, 2500)
        tibia6.attach(tibia6_pin, 500, 2500)
    except Exception as e:  # noqa: BLE001
        print(f'Error attaching servos: {e}')

    g.servos_attached = True
    # print('Servos Attached')


# AM - checked
def detach_servos():
    """Detach all servos."""
    try:
        coxa1.detach()
        femur1.detach()
        tibia1.detach()

        coxa2.detach()
        femur2.detach()
        tibia2.detach()

        coxa3.detach()
        femur3.detach()
        tibia3.detach()

        coxa4.detach()
        femur4.detach()
        tibia4.detach()

        coxa5.detach()
        femur5.detach()
        tibia5.detach()

        coxa6.detach()
        femur6.detach()
        tibia6.detach()

        g.servos_attached = False
    except Exception as e:  # noqa: BLE001
        print(f'Error detaching servos: {e}')

    # print('Servos Detached')
