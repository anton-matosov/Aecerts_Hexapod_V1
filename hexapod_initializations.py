from bezier import Vector3

# Pin definitions
coxa1_pin = 22
femur1_pin = 23
tibia1_pin = 24

coxa2_pin = 25
femur2_pin = 26
tibia2_pin = 27

coxa3_pin = 28
femur3_pin = 29
tibia3_pin = 30

coxa4_pin = 31
femur4_pin = 32
tibia4_pin = 33

coxa5_pin = 34
femur5_pin = 35
tibia5_pin = 36

coxa6_pin = 37
femur6_pin = 38
tibia6_pin = 39

# Servo pin mapping
servo_pins = [
    coxa1_pin,
    femur1_pin,
    tibia1_pin,
    coxa2_pin,
    femur2_pin,
    tibia2_pin,
    coxa3_pin,
    femur3_pin,
    tibia3_pin,
    coxa4_pin,
    femur4_pin,
    tibia4_pin,
    coxa5_pin,
    femur5_pin,
    tibia5_pin,
    coxa6_pin,
    femur6_pin,
    tibia6_pin,
]


# Hand crafted stub for Servo class
# TODO(AM): Implement the servo class to pass in the hexapod sim
class Servo:
    def attach(self, pin, min_pulse, max_pulse):
        # Placeholder for setting pulse width range
        self.pin = pin

    def detach(self):
        # Placeholder for detaching the servo
        pass

    def write_microseconds(self, microseconds):
        # Placeholder for writing pulse width in microseconds
        pass

    @staticmethod
    def __microseconds_to_angle(microseconds):
        angle = ((microseconds - 500.0) / (2500.0 - 500.0)) * 180.0
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

servos_attached = False


# AM - checked
def attach_servos():
    """Attach all servos to their pins with specified pulse width range"""
    global servos_attached

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

    servos_attached = True
    print('Servos Attached')


# AM - checked
def detach_servos():
    """Detach all servos"""
    global servos_attached

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

        servos_attached = False
    except Exception as e:  # noqa: BLE001
        print(f'Error detaching servos: {e}')

    print('Servos Detached')
