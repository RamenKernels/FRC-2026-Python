import commands
import constants
import configs
from rev import SparkMax, SparkLowLevel

class ShooterSubsystem(commands.Subsystem):
  def __init__(self) -> None:
    super.__init__();

    self.left_motor = SparkMax(constants.ShooterConstants.SHOOTER_LEFT_CAN_ID, SparkLowLevel.MotorType.kBrushless)
    self.right_motor = SparkMax(constants.ShooterConstants.SHOOTER_RIGHT_CAN_ID, SparkLowLevel.MotorType.kBrushless)

    self.left_pid = self.left_motor.getClosedLoopController()
    self.right_pid = self.right_motor.getClosedLoopController()

    self.left_motor.configure(configs.ShooterConfig.config)