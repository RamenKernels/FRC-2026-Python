import typing

from wpilib import SmartDashboard
from constants import GyroConstants, PhysicalConstants, SwerveConstants
from subsystems.swervemodule import SwerveModule
from navx import AHRS
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
)
import commands2


class SwerveSubsystem(commands2.Subsystem):
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

        modules = ["FrontLeft", "FrontRight", "BackLeft", "BackRight"]
        for name, state in zip(modules, swerve_states):
            SmartDashboard.putNumber(f"Swerve Desired States/{name} Speed", state.speed)
            SmartDashboard.putNumber(f"Swerve Desired States/{name} Angle", state.angle.degrees())

    def get_swerve_states(self) -> list:
        return [ 
                self.front_left.get_state(),
                self.front_right.get_state(),
                self.back_left.get_state(),
                self.back_right.get_state()
                ]

    def periodic(self) -> None:
        states = self.get_swerve_states()
        modules = ["FrontLeft", "FrontRight", "BackLeft", "BackRight"]
        
        for name, state in zip(modules, states):
            SmartDashboard.putNumber(f"Swerve States/{name} Speed", state.speed)
            SmartDashboard.putNumber(f"Swerve States/{name} Angle", state.angle.degrees())
