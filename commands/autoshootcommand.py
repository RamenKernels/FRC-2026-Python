import commands2

from subsystems.shootersubsystem import ShooterSubsystem


class AutoShootCommand(commands2.Command):
    def __init__(self, shooter_subsystem: ShooterSubsystem):
        super().__init__()

