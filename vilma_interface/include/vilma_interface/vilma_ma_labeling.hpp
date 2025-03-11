#ifndef VILMA_MA_LABELING__HPP_
#define VILMA_MA_LABELING__HPP_

enum class SensorsMA {
    ROS_TIME,
    TIME_PPC,
    OPERATION_STATE,
    STEER_POS,
    SPEER_SPEED,
    STEER_STATE,
    STEER_VM,
    STEER_T_USER,
    GAS_VALUE,
    GAS_USER_VALUE,
    CURRENT_SENSOR,
    ACC_COUNTER,
    ACC_X,
    ACC_Y,
    ACC_Z,
    ATMOSPHERE_PRESSURE,
    TIME_PCC_BRAKE,
    BRAKE_STATE,
    BRAKE_VALUE,
    BRAKE_WS_FRONT_LEFT,
    BRAKE_WS_FRONT_RIGHT,
    BRAKE_WS_BACK_LEFT,
    BRAKE_WS_BACK_RIGHT,
    BRAKE_WS_ACC_X,
    BRAKE_WS_ACC_Y,
    BRAKE_WS_ACC_Z,
    BRAKE_WS_GYR_Z,
    BRAKE_USER_PRESSURE,
    BRAKE_FRONT_ENCODER,
    NONE,
    GEAR_STATE,
    GEAR_OFF = 0,
    GEAR_N = 1,
    GEAR_R = 2,
    GEAR_D = 3
};

enum class StateMA {
    ROS_TIME,
    TIME_PCC,
    STEER_ANGLE,
    STEER_SPEED_ANGLE,
    STEER_TIRE_ANGLE,
    STEER_TIRE_SPEED_ANGLE,
    LATERAL_VELOCITY,
    ANGULAR_YAW_SPEED,
    X_GLOBAL_POSITION,
    Y_GLOBAL_POSITION,
    YAW_ANGLE,
    LONGITUDINAL_SPEED,
    USER_TORQUE,
    STEER_MOTOR_VOLTAGE,
    YAW_DC_ERROR
};

enum class JoystickMA {
    ROS_TIME,
    TIME_VALIDITY,
    BRAKE_COMMAND,
    BRAKE_VALUE,
    STEER_COMMAND,
    STEER_VALUE,
    GAS_COMMAND,
    GAS_VALUE,
    GEAR_STATE,
    GEAR_VALUE,
    GAS_COMMAND_OFF = 0,
    GAS_COMMAND_PEDAL_SPEED = 1,
    GAS_COMMAND_POSITION = 2,
    BRAKE_COMMAND_OFF = 0,
    BRAKE_COMMAND_AUTO = 2,
    STEER_COMMAND_OFF = 0,
    STEER_COMMAND_SPEED = 1,
    STEER_COMMAND_POSITION = 2,
    STEER_COMMAND_VOLTAGE = 3,
    STEER_COMMAND_LOOK_ZERO = 4,
    GEAR_COMMAND_OFF = 0,
    GEAR_COMMAND_NEUTRAL = 1,
    GEAR_COMMAND_REVERSE = 2,
    GEAR_COMMAND_DRIVE = 3,
    JOYSTICK_MA_DATA_LENGTH = 10
};

enum class OperationModeMA {
    INITIAL_STATE_MODE = 1,
    MANUAL_MODE = 2,
    JOYSTICK_MODE = 3
};

enum class TxTypeMA {
    SENSORS_MA = 1000,
    STATE_MA = 2000
};

enum class RxTypeMA {
    ONLY_RECEIVE_DATA = 0,
    JOYSTICK_MODE_COMMAND = 30
};


#endif  // VILMA_MA_LABELING__HPP_