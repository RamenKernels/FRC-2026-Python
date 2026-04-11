from wpimath.geometry import Translation2d
import navx
from wpimath.kinematics import SwerveDrive4Kinematics


class PhysicalConstants:
    ROBOT_MASS_POUNDS: float = 113.5
    BUMPER_LENGTH: float = 32
    TRACK_LENGTH: float = 24
    MAX_SPEED: float = 6
    MAX_ANGULAR_SPEED: float = 6
    MAX_ACCELERATION: float = 3
    MAX_ANGULAR_ACCELERATION: float = 3


class SwerveConstants:
    WHEEL_RADIUS: float = 0.0508
    ENCODER_RESOLUTION: float = 4096

    DT_SECONDS: float = 0.02

    FRONT_LEFT_DRIVE_CAN: int = 21
    FRONT_RIGHT_DRIVE_CAN: int = 11
    BACK_LEFT_DRIVE_CAN: int = 26
    BACK_RIGHT_DRIVE_CAN: int = 16

    FRONT_LEFT_TURN_CAN: int = 22
    FRONT_RIGHT_TURN_CAN: int = 12
    BACK_LEFT_TURN_CAN: int = 27
    BACK_RIGHT_TURN_CAN: int = 17

    DRIVING_MOTOR_REDUCTION: float = 8
    TURNING_MOTOR_REDUCTION: float = 21

    FRONT_LEFT_LOCATION_INCHES: Translation2d = Translation2d(12.25, 12.25)
    FRONT_RIGHT_LOCATION_INCHES: Translation2d = Translation2d(12.25, -12.25)
    BACK_LEFT_LOCATION_INCHES: Translation2d = Translation2d(12.25, -12.25)
    BACK_RIGHT_LOCATION_INCHES: Translation2d = Translation2d(12.25, -12.25)

    KINEMATICS: SwerveDrive4Kinematics = SwerveDrive4Kinematics(
        FRONT_LEFT_LOCATION_INCHES,
        FRONT_RIGHT_LOCATION_INCHES,
        BACK_LEFT_LOCATION_INCHES,
        BACK_RIGHT_LOCATION_INCHES,
    )


class ShooterConstants:
    SHOOTER_LEFT_CAN_ID: int = 41
    SHOOTER_RIGHT_CAN_ID: int = 42

    P = 0.0005;
    I = 0.0;
    D = 0.0;
    F = 0.001;


class GyroConstants:
    NAV_X_COMTYPE: navx.AHRS.NavXComType = navx.AHRS.NavXComType.kMXP_SPI


class ControllerConstants:
    MAX_THROTTLE: float = 5
    MIN_THROTTLE: float = 2.5

    FLIGHT_STICK_Y_DEADBAND: float = 0.15
    FLIGHT_STICK_X_DEADBAND: float = 0.25
    FLIGHT_STICK_Z_DEADBAND: float = 0.15

class VisionConstants:
    CAMERA_NAME: str = ""  # THIS is why you don't spend 30 minutes naming a camera!
    TRANSLATION_KP: float = 1.0
    TRANSLATION_KI: float = 0.0
    TRANSLATION_KD: float = 0.0
    ROTATION_KP: float = 1.0
    ROTATION_KI: float = 0.0
    ROTATION_KD = 0.0
