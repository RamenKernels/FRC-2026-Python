from typing import Optional
from wpilib import DriverStation
from wpimath import units
from wpimath.geometry import Rotation3d, Transform3d, Translation2d
import navx
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.trajectory import TrapezoidProfile


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

    SHOOT_THRESHOLD: float = 300

    AUTO_SHOOT_SLOPE: float = 227.501
    AUTO_SHOOT_Y_INTERCEPT: float = 1838.52

    MINUMUM_SHOOT_DISTANCE_METERS: float = 2

    P: float = 0.0005
    I: float = 0.0
    D: float = 0.0
    F: float = 0.001


class IntakeArmConstants:
    MANUAL_VOLTS: float = 1.7

    ARM_STALL_AMPS: float = 4.0

    AUTO_ARM_STALL_AMPS: float = 5.0

    INTAKE_ARM_CAN_ID: int = 51

    INTAKE_ARM_EXTEND_POWER: float = 0.5
    INTAKE_ARM_RETRACT_POWER: float = 0.5

    AUTO_VOLTS: float = 1.5


class FuelConstants:
    HOPPER_CAN_ID: int = 32
    INTAKE_CAN_ID: int = 33
    FEEDER_LEFT_CAN_ID: int = 36
    FEEDER_RIGHT_CAN_ID: int = 37
    VECTOR_CAN_ID: int = 8

    FEEDER_SPEED: float = 50
    INTAKE_SPEED: float = 1600
    HOPPER_SPEED: float = 0.5
    VECTOR_SPEED: float = 0.5

    INTAKE_P: float = 1.0
    INTAKE_I: float = 0.0
    INTAKE_D: float = 0.0

    HOPPER_P: float = 1.0
    HOPPER_I: float = 0.0
    HOPPER_D: float = 0.0

    VECTOR_P: float = 1.0
    VECTOR_I: float = 0.0
    VECTOR_D: float = 0.0


class GyroConstants:
    NAV_X_COMTYPE: navx.AHRS.NavXComType = navx.AHRS.NavXComType.kMXP_SPI


class ControllerConstants:
    MAX_THROTTLE: float = 5
    MIN_THROTTLE: float = 2.5

    FLIGHT_STICK_Y_DEADBAND: float = 0.15
    FLIGHT_STICK_X_DEADBAND: float = 0.25
    FLIGHT_STICK_Z_DEADBAND: float = 0.15


class VisionConstants:
    CAMERA_NAME: str = "cam4"

    APRIL_TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(
        AprilTagField.k2026RebuiltAndyMark
    )

    ROBOT_TO_CAMERA_1: Transform3d = Transform3d(
        units.inchesToMeters(25), 0, units.inchesToMeters(19), Rotation3d()
    )

    RED_HUB_POS: Translation2d = Translation2d(11.6741194, 4.0346376)
    BLUE_HUB_POS: Translation2d = Translation2d(4.6187614, 4.0346376)

    XY_P: float = 0.4
    XY_I: float = 0.0
    XY_D: float = 0.0
    XY_CONSTRAINTS: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
        0.01, 0.1
    )

    ROT_P: float = 1.5
    ROT_I: float = 0.0
    ROT_D: float = 0.07
    ROT_CONSTRAINTS: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
        0.25, 0.5
    )
