import commands2
from wpilib import Timer

from subsystems.fuelsubsystem import FuelSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.swervesubsystem import SwerveSubsystem
from subsystems.visionsubsystem import VisionSubsystem
import constants
from utils import mathutils, gameutils

from typing import Callable


class AutoShootCommand(commands2.Command):
    def __init__(
        self,
        shooter_subsystem: ShooterSubsystem,
        fuel_subsystem: FuelSubsystem,
        swerve_subsystem: SwerveSubsystem,
        vision_subsystem: VisionSubsystem,
        x_input: Callable[[], float],
        y_input: Callable[[], float],
        throttle: Callable[[], float],
    ):
        super().__init__()

        self.shooter_subsystem = shooter_subsystem
        self.fuel_subsystem = fuel_subsystem
        self.swerve_subsystem = swerve_subsystem
        self.vision_subsystem = vision_subsystem

        self.x_input = x_input
        self.y_input = y_input
        self.throttle = throttle

        self.addRequirements(
            shooter_subsystem, fuel_subsystem, swerve_subsystem, vision_subsystem
        )

    def can_shoot(self) -> bool:
        return (
            self.swerve_subsystem.is_aligned_with_hub()
            & self.shooter_subsystem.is_at_speed()
            & (
                self.swerve_subsystem.get_hub_distance()
                > constants.ShooterConstants.MINUMUM_SHOOT_DISTANCE_METERS
            )
        )

    def initialize(self) -> None:
        self.shooter_subsystem.shoot(
            self.shooter_subsystem.get_shoot_power(
                self.swerve_subsystem.get_hub_distance()
            )
        )

    def execute(self):
        if self.can_shoot():
            self.fuel_subsystem.feed()
            self.shooter_subsystem.shoot(
                self.shooter_subsystem.get_shoot_power(
                    self.swerve_subsystem.get_hub_distance()
                )
            )

        self.swerve_subsystem.align_to_and_drive(
            self.x_input(),
            self.y_input(),
            mathutils.angle_to(
                self.swerve_subsystem.get_pose().translation(), gameutils.get_hub_pose()
            ),
            False,
        )

        if self.vision_subsystem.has_valid_target():
            self.swerve_subsystem.add_vision_measurement(
                self.vision_subsystem.get_last_average_global_pose(),
                Timer.getFPGATimestamp(),
            )

    def end(self, interrupted: bool):
        self.fuel_subsystem.stop()
        self.shooter_subsystem.stop()
