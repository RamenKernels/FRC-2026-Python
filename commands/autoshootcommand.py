import commands2

from subsystems.fuelsubsystem import FuelSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.swervesubsystem import SwerveSubsystem


class AutoShootCommand(commands2.Command):
    def __init__(self, shooter_subsystem: ShooterSubsystem, fuel_subsystem: FuelSubsystem, swerve_subsystem: SwerveSubsystem, vision_subsystem VisionSubsystem):
        super().__init__()

