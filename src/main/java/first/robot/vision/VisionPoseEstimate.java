package first.robot.vision;

import org.photonvision.EstimatedRobotPose;
import org.wpilib.math.geometry.Pose2d;

/**
 * Container for pose estimates.
 */
public class VisionPoseEstimate {
    public final Pose2d pose2d;
    public final double timestampSeconds;

    public VisionPoseEstimate(Pose2d pose2d, double timestampSeconds) {
        this.pose2d = pose2d;
        this.timestampSeconds = timestampSeconds;
    }

    public VisionPoseEstimate(EstimatedRobotPose estimatedRobotPose) {
        this(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
    }

    @Override
    public String toString() {
        return String.format("VisionPoseEstimate(%s, %.3f)", this.pose2d, this.timestampSeconds);
    }
}
