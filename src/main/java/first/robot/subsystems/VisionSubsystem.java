package first.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.system.Timer;

import first.robot.util.DebouncedBoolean;
import first.robot.vision.AprilTag;
import first.robot.vision.PhotonVisionCamera;
import first.robot.vision.VisionCamera;
import first.robot.vision.VisionPoseEstimate;

/**
 * Interfaces with PhotonVision, updating the {@code DriveSubsystem} odometry
 * whenever an AprilTag is seen. Also allows Commands to determine if specific
 * AprilTags are visible.
 */
public class VisionSubsystem extends SubsystemBase {

    private final DebouncedBoolean isTargetVisible = new DebouncedBoolean(200L);
    private final DriveSubsystem driveSubsystem;
    public final NetworkTableEntry apriltagsVisibleDisplay = NetworkTableInstance.getDefault()
            .getEntry("VisionSubsystem/AprilTagsVisible");
    public final NetworkTableEntry periodicTimeDisplay = NetworkTableInstance.getDefault()
            .getEntry("VisionSubsystem/ProcessingTime");
    public final StringPublisher apriltagLogger = NetworkTableInstance.getDefault()
            .getStringTopic("SmartDashboard/VisionSubsystem/AprilTagsCamera").publish();
    public final List<VisionCamera> cameraList = new ArrayList<>();

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        cameraList.add(new PhotonVisionCamera("Arducam_OV9281_USB_Camera",
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(0.0), // left or right from center
                                Units.inchesToMeters(8.0), // forward from robot center
                                Units.inchesToMeters(12.0)), // distance up from the floor
                        new Rotation3d(
                                Rotation2d.fromDegrees(0).getRadians(), // roll
                                Rotation2d.fromDegrees(0.0).getRadians(), // pitch
                                Rotation2d.fromDegrees(0).getRadians())))); // yaw

    }

    @Override
    public void periodic() {
        final double time = Timer.getTimestamp();
        boolean targetFound = false;
        for (VisionCamera visionCamera : cameraList) {
            VisionPoseEstimate estimatedRobotPose = visionCamera.getEstimatedRobotPose();
            if (estimatedRobotPose != null) {
                targetFound = true;
                if (estimatedRobotPose.timestampSeconds != visionCamera.getPrevTimeStamp()) {
                    driveSubsystem.addVisionMeasurement(
                            estimatedRobotPose.pose2d,
                            estimatedRobotPose.timestampSeconds);
                    apriltagLogger.set(visionCamera.toString());
                }
                visionCamera.setPrevTimestamp(estimatedRobotPose.timestampSeconds);
            }
        }
        isTargetVisible.set(targetFound);
        apriltagsVisibleDisplay.setBoolean(isTargetVisible.get());
        periodicTimeDisplay.setInteger(Math.round(1000 * (Timer.getTimestamp() - time)));
    }

    /**
     * @return whether any AprilTag is visible.
     */
    public boolean cameraSeesTargets() {
        return isTargetVisible.get();
    }

    /**
     * Find the best {@code AprilTag} currently visible for a given list of IDs.
     * If no fiducial IDs are specified, then return any possible tag.
     *
     * @param fiducialIDs optional list of tag IDs.
     * @return the best visible {@code AprilTag} for any of the fiducialIDs.
     */
    public AprilTag getBestTag(Integer... fiducialIDs) {
        AprilTag aprilTag = null;
        for (VisionCamera visionCamera : cameraList) {
            aprilTag = visionCamera.getBestTag(fiducialIDs);
            if (aprilTag != null) {
                break;
            }
        }
        return aprilTag;
    }
}
