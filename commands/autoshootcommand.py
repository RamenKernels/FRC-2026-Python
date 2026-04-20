import commands2
from wpilib import SmartDashboard, Timer

from subsystems.fuelsubsystem import FuelSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.swervesubsystem import SwerveSubsystem
from subsystems.visionsubsystem import VisionSubsystem

from typing import Callable


class AutoShootCommand(commands2.Command):
    def __init__(self,
                 shooter_subsystem: ShooterSubsystem,
                 fuel_subsystem: FuelSubsystem,
                 swerve_subsystem: SwerveSubsystem,
                 vision_subsystem: VisionSubsystem,
                 x_input: Callable[[], float],
                 y_input: Callable[[], float],
                 throttle: Callable[[], float]):
        super().__init__()

        self.shooter_subsystem = shooter_subsystem
        self.fuel_subsystem = fuel_subsystem
        self.swerve_subsystem = swerve_subsystem
        self.vision_subsystem = vision_subsystem

        self.x_input = x_input
        self.y_input = y_input
        self.throttle = throttle

        self.timer = Timer()

        self.addRequirements(shooter_subsystem, fuel_subsystem, swerve_subsystem, vision_subsystem)

    def can_shoot(self) -> bool:
        SmartDashboard("Is aligned", s



