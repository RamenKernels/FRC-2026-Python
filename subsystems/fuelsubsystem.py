import commands2
from rev import SparkLowLevel, SparkMax

import constants
import configs


class FuelSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.intake_motor = SparkMax(constants.FuelConstants.INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless)
        self.hopper_motor = SparkMax(constants.FuelConstants.HOPPER_CAN_ID, SparkLowLevel.MotorType.kBrushless)
        self.vector_motor = SparkMax(constants.FuelConstants.VECTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless)
        self.feeder_left_motor = SparkMax(constants.FuelConstants.FEEDER_LEFT_CAN_ID, SparkLowLevel.MotorType.kBrushless)
        self.feeder_right_motor = SparkMax(constants.FuelConstants.FEEDER_RIGHT_CAN_ID, SparkLowLevel.MotorType.kBrushless)

        self.feeder_left_motor.configure(configs.FeederConfig

