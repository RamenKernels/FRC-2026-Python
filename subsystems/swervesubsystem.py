import typing

from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import math
from constants import GyroConstants, PhysicalConstants, SwerveConstants
import constants
from subsystems.swervemodule import SwerveModule
from navx import AHRS
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
)
import commands2
from utils import gameutils, mathutils


class SwerveSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._front_left = SwerveModule(
            SwerveConstants.FRONT_LEFT_DRIVE_CAN, SwerveConstants.FRONT_LEFT_TURN_CAN
        )
        self._front_right = SwerveModule(
            SwerveConstants.FRONT_RIGHT_DRIVE_CAN, SwerveConstants.FRONT_RIGHT_TURN_CAN
        )
        self._back_left = SwerveModule(
            SwerveConstants.BACK_LEFT_DRIVE_CAN, SwerveConstants.BACK_LEFT_TURN_CAN
        )
        self._back_right = SwerveModule(
            SwerveConstants.BACK_RIGHT_DRIVE_CAN, SwerveConstants.BACK_RIGHT_TURN_CAN
        )

        self._gyro = AHRS(GyroConstants.NAV_X_COMTYPE)

        self._x_pid = PIDController(
            constants.VisionConstants.XY_P,
            constants.VisionConstants.XY_I,
            constants.VisionConstants.XY_D,
        )
        self._y_pid = PIDController(
            constants.VisionConstants.XY_P,
            constants.VisionConstants.XY_I,
            constants.VisionConstants.XY_D,
        )
        self._rot_pid = PIDController(
            constants.VisionConstants.ROT_P,
            constants.VisionConstants.ROT_I,
            constants.VisionConstants.ROT_D,
        )

        self._rot_pid.enableContinuousInput(-math.pi, math.pi)

        self._pose_estimator = SwerveDrive4PoseEstimator(
            constants.SwerveConstants.KINEMATICS,
            self.get_rotation(),
            tuple(self.get_module_positions()),
            Pose2d(),
        )

    def drive(self, x: float, y: float, rot: float, field_oriented: bool) -> None:
        heading = self.get_pose().rotation()

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            heading = heading.rotateBy(Rotation2d(math.pi))

        chassis_speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x * PhysicalConstants.MAX_SPEED,
                y * PhysicalConstants.MAX_SPEED,
                rot * PhysicalConstants.MAX_ANGULAR_SPEED,
                heading,
            )
            if field_oriented
            else ChassisSpeeds(
                x * PhysicalConstants.MAX_SPEED,
                y * PhysicalConstants.MAX_SPEED,
                rot * PhysicalConstants.MAX_ANGULAR_SPEED,
            )
        )

        self.set_chassis_speeds(chassis_speeds)

    def drive_relative_to(self, current_pose: Pose2d, new_pose: Pose2d) -> None:
        x_out: float = -self._x_pid.calculate(current_pose.X(), new_pose.X())
        y_out: float = -self._y_pid.calculate(current_pose.Y(), new_pose.Y())
        rot_out: float = self._rot_pid.calculate(current_pose.rotation().radians())

        self.drive(x_out, y_out, rot_out, False)

    def drive_to(self, pose: Pose2d) -> None:
        x_out: float = -self._x_pid.calculate(self.get_pose().X(), pose.X())
        y_out: float = -self._y_pid.calculate(self.get_pose().Y(), pose.Y())
        rot_out: float = self._rot_pid.calculate(
            self.get_pose().rotation().radians(), pose.rotation().radians()
        )

        self.drive(x_out, y_out, rot_out, False)

    def align_to_and_drive(
        self, x: float, y: float, target_rotation: Rotation2d, field_oriented: bool
    ) -> None:
        rot_out = -self._rot_pid.calculate(
            self.get_pose().rotation().radians(), target_rotation.radians()
        )
        self.drive(x, y, rot_out, field_oriented)

    def get_pose(self) -> Pose2d:
        return self._pose_estimator.getEstimatedPosition()

    def get_rotation(self) -> Rotation2d:
        return self._gyro.getRotation2d()

    def get_module_positions(self) -> list[SwerveModulePosition]:
        return [
            self._front_left.get_position(),
            self._front_right.get_position(),
            self._back_left.get_position(),
            self._back_right.get_position(),
        ]

    def get_hub_distance(self) -> float:
        return gameutils.get_hub_pose().distance(self.get_pose().translation())

    def set_chassis_speeds(self, chassis_speeds: ChassisSpeeds) -> None:
        swerve_states = SwerveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassis_speeds, SwerveConstants.DT_SECONDS)
        )

        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_states, PhysicalConstants.MAX_SPEED
        )

        self._front_left.set_desired_state(swerve_states[0])
        self._front_right.set_desired_state(swerve_states[1])
        self._back_left.set_desired_state(swerve_states[2])
        self._back_right.set_desired_state(swerve_states[3])

    def add_vision_measurement(self, measurement: Pose2d, timestamp: float) -> None:
        self._pose_estimator.addVisionMeasurement(measurement, timestamp)

    def is_aligned_with_hub(self) -> bool:
        if DriverStation.getAlliance is not None:
            return math.isclose(
                mathutils.angle_to(
                    gameutils.get_hub_pose(), self.get_pose().translation()
                ),
                0.05,
            )
        return False

    def periodic(self) -> None:
        self._pose_estimator.update(
            self.get_rotation(), tuple(self.get_module_positions())
        )
