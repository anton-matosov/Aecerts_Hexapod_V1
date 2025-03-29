from adafruit_servokit import ServoKit
from vector3 import Vector3

# Initialize ServoKit for controlling servos
kit = ServoKit(channels=16)  # Adjust channels as needed, might need multiple controllers

# Individual servo objects
coxa1 = kit.servo[0]
femur1 = kit.servo[1]
tibia1 = kit.servo[2]

coxa2 = kit.servo[3]
femur2 = kit.servo[4]
tibia2 = kit.servo[5]

coxa3 = kit.servo[6]
femur3 = kit.servo[7]
tibia3 = kit.servo[8]

coxa4 = kit.servo[9]
femur4 = kit.servo[10]
tibia4 = kit.servo[11]

coxa5 = kit.servo[12]
femur5 = kit.servo[13]
tibia5 = kit.servo[14]

coxa6 = kit.servo[15]
femur6 = kit.servo[16]
tibia6 = kit.servo[17]

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

# Leg dimensions
a1 = 46.0  # Coxa Length
a2 = 108.0  # Femur Length
a3 = 200.0  # Tibia Length
leg_length = a1 + a2 + a3

# Position tracking
current_points = [Vector3() for _ in range(6)]
cycle_start_points = [Vector3() for _ in range(6)]

current_rot = Vector3(180, 0, 180)
target_rot = Vector3(180, 0, 180)

stride_multiplier = [1, 1, 1, -1, -1, -1]
rotation_multiplier = [-1, 0, 1, -1, 0, 1]

control_points = [Vector3() for _ in range(10)]
rotate_control_points = [Vector3() for _ in range(10)]
attack_control_points = [Vector3() for _ in range(10)]

servos_attached = False


def attach_servos():
    """Attach all servos to their pins with specified pulse width range"""
    global servos_attached

    try:
        coxa1.set_pulse_width_range(500, 2500)
        femur1.set_pulse_width_range(500, 2500)
        tibia1.set_pulse_width_range(500, 2500)

        coxa2.set_pulse_width_range(500, 2500)
        femur2.set_pulse_width_range(500, 2500)
        tibia2.set_pulse_width_range(500, 2500)

        coxa3.set_pulse_width_range(500, 2500)
        femur3.set_pulse_width_range(500, 2500)
        tibia3.set_pulse_width_range(500, 2500)

        coxa4.set_pulse_width_range(500, 2500)
        femur4.set_pulse_width_range(500, 2500)
        tibia4.set_pulse_width_range(500, 2500)

        coxa5.set_pulse_width_range(500, 2500)
        femur5.set_pulse_width_range(500, 2500)
        tibia5.set_pulse_width_range(500, 2500)

        coxa6.set_pulse_width_range(500, 2500)
        femur6.set_pulse_width_range(500, 2500)
        tibia6.set_pulse_width_range(500, 2500)
    except Exception as e:
        print(f'Error attaching servos: {e}')

    servos_attached = True
    print('Servos Attached')


def detach_servos():
    """Detach all servos"""
    global servos_attached

    # In Python with ServoKit, we might not have a direct "detach" method
    # We could set servos to a neutral position or disable them
    try:
        # Implementation depends on your servo library
        # This is a placeholder - you may need to adjust based on your hardware
        pass
    except Exception as e:
        print(f'Error detaching servos: {e}')

    servos_attached = False
    print('Servos Detached')
