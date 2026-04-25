import commands2
from rev import PersistMode, ResetMode, SparkLowLevel, SparkMax

import constants
import configs


class FuelSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.intake_motor = SparkMax(
            constants.FuelConstants.INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless
        )
        self.hopper_motor = SparkMax(
            constants.FuelConstants.HOPPER_CAN_ID, SparkLowLevel.MotorType.kBrushless
        )
        self.vector_motor = SparkMax(
            constants.FuelConstants.VECTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless
        )
        self.feeder_motors = SparkMax(
            constants.FuelConstants.FEEDER_LEFT_CAN_ID,
            SparkLowLevel.MotorType.kBrushless,
        )  # Left motor
        self.feeder_follower_motor = SparkMax(
            constants.FuelConstants.FEEDER_RIGHT_CAN_ID,
            SparkLowLevel.MotorType.kBrushless,
        )  # right motor

        self.intake_motor.configure(
            configs.IntakeConfig.config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )
        self.hopper_motor.configure(
            configs.HopperConfig.config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )
        self.vector_motor.configure(
            configs.VectorConfig.config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )

        self.feeder_motors.configure(
            configs.FeederConfig.left_config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )  # Left motor
        self.feeder_follower_motor.configure(
            configs.FeederConfig.right_config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )  # right motor

    def intake(self) -> None:
        self.intake_motor.set(constants.FuelConstants.INTAKE_SPEED)
        self.hopper_motor.set(constants.FuelConstants.HOPPER_SPEED)

    def outtake(self) -> None:
        self.intake_motor.set(-constants.FuelConstants.INTAKE_SPEED)
        self.hopper_motor.set(-constants.FuelConstants.HOPPER_SPEED)
        self.vector_motor.set(-constants.FuelConstants.VECTOR_SPEED)
        self.feeder_motors.set(-constants.FuelConstants.FEEDER_SPEED)

    def feed(self) -> None:
        self.hopper_motor.set(constants.FuelConstants.HOPPER_SPEED)
        self.vector_motor.set(constants.FuelConstants.VECTOR_SPEED)
        self.feeder_motors.set(constants.FuelConstants.FEEDER_SPEED)

    def stop(self) -> None:
        self.intake_motor.stopMotor()
        self.hopper_motor.stopMotor()
        self.vector_motor.stopMotor()
        self.feeder_motors.stopMotor()

    def intakeCommand(self) -> commands2.Command:
        return commands2.cmd.startEnd(lambda: self.intake(), lambda: self.stop())

    def outtakeCommand(self) -> commands2.Command:
        return commands2.cmd.startEnd(lambda: self.outtake(), lambda: self.stop())

    def feedCommand(self) -> commands2.Command:
        return commands2.cmd.startEnd(lambda: self.feed(), lambda: self.stop())
