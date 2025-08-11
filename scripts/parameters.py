
N_JOINTS = 10

DEVICE = [
        "can1", "can1", "can1", "can1", "can1",
        "can0", "can0", "can0", "can0", "can0",
        ]

CAN_ID = [
        6, 7, 8, 9, 10,
        1, 2, 3, 4, 5,
        ]

MOTOR_TYPE = [
        "AK70_10_V1p1", "AK70_10_V1p1", "AK10_9_V1p1", "AK10_9_V1p1", "AK70_10_V1p1",
        "AK70_10_V1p1", "AK70_10_V1p1", "AK10_9_V1p1", "AK10_9_V1p1", "AK70_10_V1p1",
        ]

JOINT_NAME = [
        'L_hip_y','L_hip_r', 'L_hip_p', 'L_knee_p', 'L_ankle_p',
        'R_hip_y','R_hip_r', 'R_hip_p', 'R_knee_p', 'R_ankle_p',
        ]

MOTOR_DIR = [
         1,  1, -1, -1,  1,
         1,  1,  1,  1, -1,
        ]

STANDBY_ANGLE = [
         0, 0, -3.09481, 2.76687, -1.34321,
         0, 0, -3.09481, 2.76687, -1.34321,
        ]

STANDBY2_ANGLE = [
         0, 0, -1.0, 2.1, -1.05,
         0, 0, -1.0, 2.1, -1.05,
        ]

DEFAULT_ANGLE = [
         0, 0, -0.5, 1.0, -0.5,
         0, 0, -0.5, 1.0, -0.5,
         # 0, 0, -1.2, 2.1, -1.05,
         # 0, 0, -1.2, 2.1, -1.05,
        ]

KP_GAIN = [
        70.0, 100.0, 100.0, 100.0, 30.0,
        70.0, 100.0, 100.0, 100.0, 30.0,
        ]

KD_GAIN = [
        2.0, 3.0, 3.0, 3.0, 0.5,
        2.0, 3.0, 3.0, 3.0, 0.5,
        ]

ACTION_SCALE = 0.2
ACTION_CLIPPING = 20
CONTROL_DECIMATION = 4
CONTROL_DT = 0.005

CONTROL_HZ = 50

CAN_HZ = 200

SIM_HZ = 200

