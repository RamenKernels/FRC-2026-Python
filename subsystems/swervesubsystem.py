import typing
from constants import GyroConstants, PhysicalConstants, SwerveConstants
from swervemodule import SwerveModule
from navx import AHRS
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
)
import commands 


class SwerveSubsystem(commands.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.front_left = SwerveModule(
            SwerveConstants.FRONT_LEFT_DRIVE_CAN, SwerveConstants.FRONT_LEFT_TURN_CAN
        )
        self.front_right = SwerveModule(
            SwerveConstants.FRONT_RIGHT_DRIVE_CAN, SwerveConstants.FRONT_RIGHT_TURN_CAN
        )
        self.back_left = SwerveModule(
            SwerveConstants.BACK_LEFT_DRIVE_CAN, SwerveConstants.BACK_LEFT_TURN_CAN
        )
        self.back_right = SwerveModule(
            SwerveConstants.BACK_RIGHT_DRIVE_CAN, SwerveConstants.BACK_RIGHT_TURN_CAN
        )

        self.gyro = AHRS(GyroConstants.NAV_X_COMTYPE)

    def drive(self, x: float, y: float, rot: float, field_oriented: bool) -> None:
        swerve_speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x * PhysicalConstants.MAX_SPEED,
                y * PhysicalConstants.MAX_SPEED,
                rot * PhysicalConstants.MAX_ANGULAR_SPEED,
                self.gyro.getRotation2d(),
            )
            if field_oriented
            else ChassisSpeeds(
                x * PhysicalConstants.MAX_SPEED,
                y * PhysicalConstants.MAX_SPEED,
                rot * PhysicalConstants.MAX_ANGULAR_SPEED,
            )
        )
        swerve_states = SwerveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.discretize(swerve_speeds, SwerveConstants.DT_SECONDS)
        )

        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_states, PhysicalConstants.MAX_SPEED
        )

        self.front_left.set_desired_state(swerve_states[0])
        self.front_right.set_desired_state(swerve_states[1])
        self.back_left.set_desired_state(swerve_states[2])
        self.back_right.set_desired_state(swerve_states[3])

    def driveCommand(self, 
                     drive: typing.Callable[[], float],
                     strafe: typing.Callable[[], float],
                     turn: typing.Callable[[], float],
                     throttle: typing.Callable[[], float],
                     field_oriented: typing.Callable[[], bool],
                    ):
        self.drive(drive * throttle(), strafe * throttle(), turn * throttle(), field_oriented)