import math
from rev import SparkMaxConfig, FeedbackSensor
from constants import SwerveConstants
import constants


class SwerveConfig:
    drive_config = SparkMaxConfig()
    turn_config = SparkMaxConfig()

    driving_factor = (
        SwerveConstants.WHEEL_RADIUS
        * 2
        * math.pi
        / SwerveConstants.DRIVING_MOTOR_REDUCTION
    )
    turning_factor = math.pi * 2

    driving_velocity_ff = 1 / 6

    drive_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(
        50
    ).inverted(True)
    drive_config.encoder.positionConversionFactor(
        driving_factor
    ).velocityConversionFactor(driving_factor * 60)
    drive_config.closedLoop.setFeedbackSensor(
        FeedbackSensor.kPrimaryEncoder
    ).pid(0.4, 0, 0).velocityFF(driving_velocity_ff).outputRange(-1, 1)

    turn_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(
        20
    ).inverted(True)
    turn_config.absoluteEncoder.inverted(True).positionConversionFactor(
        turning_factor
    ).velocityConversionFactor(turning_factor / 60)
    turn_config.closedLoop.pid(1, 0, 0).setFeedbackSensor(
        FeedbackSensor.kAbsoluteEncoder
    ).outputRange(-1, 1).positionWrappingEnabled(True).positionWrappingInputRange(
        0, turning_factor
    )


class FuelConfig:
    hopper_config = SparkMaxConfig()
    vector_config = SparkMaxConfig()
    feeder_config = SparkMaxConfig()
    



class ShooterConfig:
    left_config = SparkMaxConfig()
    right_config = SparkMaxConfig()

    left_config\
            .setIdleMode(SparkMaxConfig.IdleMode.kCoast)\
            .smartCurrentLimit(100)\
            .inverted(True)
    left_config.closedLoop.\
            setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)\
            .pid(constants.ShooterConstants.P,
                 constants.ShooterConstants.I,
                 constants.ShooterConstants.D)\
                         .feedForward.kV(constants.ShooterConstants.F)

    #Make the right follow the left but inverted
    right_config = left_config.follow(constants.ShooterConstants.SHOOTER_LEFT_CAN_ID, True)
