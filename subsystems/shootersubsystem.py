import commands2
import wpilib
from wpimath.units import math
from wpiutil import typing
import constants
import configs
from rev import PersistMode, ResetMode, SparkMax, SparkLowLevel


class ShooterSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.left_motor = SparkMax(
            constants.ShooterConstants.SHOOTER_LEFT_CAN_ID,
            SparkLowLevel.MotorType.kBrushless,
        )
        self.right_motor = SparkMax(
            constants.ShooterConstants.SHOOTER_RIGHT_CAN_ID,
            SparkLowLevel.MotorType.kBrushless,
        )

        self.left_motor.configure(
            configs.ShooterConfig.config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )
        self.right_motor.configure(
            configs.ShooterConfig.config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )

        self.left_pid = self.left_motor.getClosedLoopController()

        self.target_shoot_speed = 0

    def shoot(self, shoot_speed_rpm: float) -> None:

        self.left_pid.setSetpoint(shoot_speed_rpm, SparkLowLevel.ControlType.kVelocity)

    def stop(self) -> None:
        self.left_motor.stopMotor()

    def is_at_speed(self) -> bool:
        return math.isclose(
            self.target_shoot_speed + 100,
            self.left_motor.getEncoder().getVelocity(),
            rel_tol=constants.ShooterConstants.SHOOT_THRESHOLD,
        )

    def get_shoot_power(self, distance: float) -> float:
        return (
            constants.ShooterConstants.AUTO_SHOOT_SLOPE * distance
            + constants.ShooterConstants.AUTO_SHOOT_Y_INTERCEPT
        )

    def shoot_command(self, speed: typing.Callable) -> commands2.Command:
        return commands2.cmd.startEnd(lambda: self.shoot(speed()), lambda: self.stop())
