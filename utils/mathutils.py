import math
from wpimath.geometry import Rotation2d, Translation2d


def angle_to(point1: Translation2d, point2: Translation2d) -> Rotation2d:
    return Rotation2d(math.atan2(point2.Y() - point1.Y(), point2.X() - point1.X()))
