import commands2
from rev import PersistMode, ResetMode, SparkLowLevel, SparkMax
from typing import Callable

import configs
import constants


class IntakeArmSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.arm_motor = SparkMax(
            constants.IntakeArmConstants.INTAKE_ARM_CAN_ID,
            SparkLowLevel.MotorType.kBrushless,
        )
        self.arm_encoder = self.arm_motor.getEncoder()
        self.arm_pid = self.arm_motor.getClosedLoopController()

        self.arm_motor.configure(
            configs.IntakeArmConfig.config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )

    def reset_encoder(self) -> None:
        self.arm_encoder.setPosition(0)

    def get_encoder_position(self) -> float:
        return self.arm_encoder.getPosition()

    def stopIntakeArm(self) -> None:
        self.arm_motor.stopMotor()

    def setVolts(self, volts: float) -> None:
        self.arm_motor.set(volts)

    def manual_arm_command(self, power: Callable[[], float]) -> commands2.Command:
        return commands2.cmd.runEnd(
            lambda: self.setVolts(power() * constants.IntakeArmConstants.MANUAL_VOLTS),
            lambda: self.stopIntakeArm(),
        )
