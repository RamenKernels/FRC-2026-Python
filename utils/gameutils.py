from typing import Optional
from wpilib import DriverStation
from wpimath.geometry import Translation2d

from constants import VisionConstants


def get_hub_pose() -> Translation2d:
    alliance: Optional[DriverStation.Alliance] = DriverStation.getAlliance()

    if alliance is not None:
        if alliance == DriverStation.Alliance.kRed:
            return VisionConstants.RED_HUB_POS
        elif alliance == DriverStation.Alliance.kBlue:
            return VisionConstants.BLUE_HUB_POS

    return VisionConstants.BLUE_HUB_POS
