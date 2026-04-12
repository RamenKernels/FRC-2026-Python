import commands2
import typing
from subsystems.swervesubsystem import SwerveSubsystem

class DriveCommand(commands2.Command):
    def __init__(self,
                 swerve_subsystem: SwerveSubsystem,
                 drive: typing.Callable[[], float],
                 strafe: typing.Callable[[], float],
                 turn: typing.Callable[[], float],
                 throttle: typing.Callable[[], float],
                 field_oriented: typing.Callable[[], bool]) -> None:
        super().__init__()

        self.swerve_subsystem = swerve_subsystem
        self.drive = drive
        self.strafe = strafe
        self.turn = turn
        self.throttle = throttle
        self.field_oriented = field_oriented

        self.addRequirements(self.swerve_subsystem)

    def execute(self) -> None:
        x = self.drive() * self.throttle()
        y = self.strafe() * self.throttle()
        rot = self.turn() * self.throttle()

        self.swerve_subsystem.drive(x, y, rot, self.field_oriented())

    def end(self, interrupted: bool) -> None:
        self.swerve_subsystem.drive(0, 0, 0, False)
