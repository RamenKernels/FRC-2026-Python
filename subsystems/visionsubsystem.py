from typing import Optional
import commands2
from photonlibpy import EstimatedRobotPose, PhotonCamera, PhotonPoseEstimator
from photonlibpy.targeting import PhotonPipelineResult, PhotonTrackedTarget
from wpilib import SmartDashboard
from wpimath.filter import LinearFilter
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Transform3d
import constants


class VisionSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.camera = PhotonCamera(constants.VisionConstants.CAMERA_NAME)

        self.photon_pose_estimator = PhotonPoseEstimator(
            constants.VisionConstants.APRIL_TAG_LAYOUT,
            constants.VisionConstants.ROBOT_TO_CAMERA_1,
        )

        self.x_measurement_filter: LinearFilter = LinearFilter.movingAverage(20)
        self.y_measurement_filter: LinearFilter = LinearFilter.movingAverage(20)
        self.rot_measurement_filter: LinearFilter = LinearFilter.movingAverage(20)

        self.result: PhotonPipelineResult = PhotonPipelineResult()

        self.robot_to_target = Transform3d()

    def has_valid_target(self) -> bool:
        return self.result.hasTargets()

    def get_estimated_global_pose(self) -> Optional[EstimatedRobotPose]:
        vision_estimate = self.photon_pose_estimator.estimateCoprocMultiTagPose(
            self.result
        )
        if vision_estimate is None:
            vision_estimate = self.photon_pose_estimator.estimateLowestAmbiguityPose(
                self.result
            )

        return vision_estimate

    def get_last_average_global_pose(self) -> Pose2d:
        return Pose2d(
            self.x_measurement_filter.lastValue(),
            self.y_measurement_filter.lastValue(),
            Rotation2d(self.rot_measurement_filter.lastValue()),
        )

    def get_estimated_relative_pose(self) -> Pose2d | None:
        target = self.result.getBestTarget()

        if target is None:
            return None

        camera_to_tag: Transform3d = target.getBestCameraToTarget()
        tag_to_camera: Transform3d = camera_to_tag.inverse()

        camera_pose: Pose3d = Pose3d(
            tag_to_camera.X(),
            tag_to_camera.Y(),
            tag_to_camera.Z(),
            tag_to_camera.rotation(),
        )

        robot_pose: Pose3d = camera_pose.transformBy(
            constants.VisionConstants.ROBOT_TO_CAMERA_1.inverse()
        )

        return robot_pose.toPose2d()

    def periodic(self) -> None:
        results = self.camera.getAllUnreadResults()

        results = [
            r
            for r in results
            if (target := r.getBestTarget()) is not None
            and target.getPoseAmbiguity() <= 0.5
        ]

        if not results:
            return

        self.result = results[-1]
        if self.result.hasTargets():
            best_target = self.result.getBestTarget()
            assert best_target is not None
            self.robot_to_target = (
                constants.VisionConstants.ROBOT_TO_CAMERA_1
                + best_target.getBestCameraToTarget()
            )

            if (estimated_pose := self.get_estimated_global_pose()) is not None:
                assert estimated_pose is not None

                self.x_measurement_filter.calculate(estimated_pose.estimatedPose.X())
                self.y_measurement_filter.calculate(estimated_pose.estimatedPose.Y())
                self.rot_measurement_filter.calculate(
                    estimated_pose.estimatedPose.rotation().toRotation2d().degrees()
                )
