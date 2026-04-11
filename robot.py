#!/usr/bin/env python3

import os
import typing

import wpilib
import commands2

from robotcontainer import RobotContainer

os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"


class MyRobot(commands2.TimedCommandRobot):
  autonomousCommand: typing.Optional[commands2.Command] = None

  def robotInit(self) -> None:
    self.container = RobotContainer()

  def disabledInit(self) -> None:
    pass

  def disabledPeriodic(self) -> None:
    pass

  def autonomousInit(self) -> None:
    self.autonomousCommand = self.container.getAutonomousCommand()

    if self.autonomousCommand:
      self.autonomousCommand.schedule()

  def autonomousPeriodic(self) -> None:
    pass

  def teleopInit(self) -> None:
    if self.autonomousCommand:
      self.autonomousCommand.cancel()

  def teleopPeriodic(self) -> None:
    pass

  def testInit(self) -> None:
    commands2.CommandScheduler.getInstance().cancelAll()
