import typing

import commands2
import wpilib
from subsystems.swervesubsystem import SwerveSubsystem
from commands.drivecommand import DriveCommand


class RobotContainer:
    def __init__(self) -> None:
        self.swerve_subsystem = SwerveSubsystem()

        self.joystick = commands2.button.CommandJoystick(0)
        self.xbox_controller = commands2.button.CommandXboxController(1)

        self.chooser = wpilib.SendableChooser()

        self._configure_button_bindings()

    def _configure_button_bindings(self):
        self.swerve_subsystem.setDefaultCommand(
            DriveCommand(
                self.swerve_subsystem,
                lambda: self.joystick.getY(),
                lambda: self.joystick.getX(),
                lambda: self.joystick.getZ(),
                lambda: 1,
                # lambda: self.joystick.getThrottle(),
                self.joystick.button(1),
            )
        )
        wpilib.SmartDashboard.putData(self.chooser)

    def getAutonomousCommand(self) -> typing.Optional[commands2.Command]:
        return self.chooser.getSelected()
