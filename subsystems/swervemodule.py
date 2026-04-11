from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from rev import SparkMax, SparkLowLevel
from constants import SwerveConstants


class SwerveModule:
    def __init__(self, drive_can_id: int, turn_can_id: int) -> None:
        self.drive_motor = SparkMax(drive_can_id, SparkLowLevel.MotorType.kBrushless)
        self.turn_motor = SparkMax(turn_can_id, SparkLowLevel.MotorType.kBrushless)

        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder = self.turn_motor.getAbsoluteEncoder()

        self.drive_pid = self.drive_motor.getClosedLoopController()
        self.turn_pid = self.turn_motor.getClosedLoopController()

        self.drive_encoder.setPosition(0)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.drive_encoder.getVelocity(),
            Rotation2d(self.turn_encoder.getPosition()),
        )

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.drive_encoder.getPosition(),
            Rotation2d(self.turn_encoder.getPosition()),
        )

    def get_drive_temp(self) -> float:
        return self.drive_motor.getMotorTemperature()

    def get_turn_temp(self) -> float:
        return self.turn_motor.getMotorTemperature()

    def set_desired_state(self, desired_state: SwerveModuleState) -> None:
        encoder_rotation = Rotation2d(self.turn_encoder.getPosition())

        desired_state.optimize(encoder_rotation)

        desired_state.cosineScale(encoder_rotation)

        self.drive_pid.setReference(
            desired_state.speed, SparkLowLevel.ControlType.kVelocity
        )
        self.turn_pid.setReference(
            desired_state.angle.radians(), SparkLowLevel.ControlType.kPosition
        )
