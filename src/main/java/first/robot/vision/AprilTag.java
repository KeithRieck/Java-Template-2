package first.robot.vision;

import org.wpilib.math.geometry.Pose3d;

/** 
 * Represents one AprilTag on the field. 
 */
public class AprilTag {
    private final int ID;
    private final Pose3d pose;

    public AprilTag(int ID, Pose3d pose) {
        this.ID = ID;
        this.pose = pose;
    }

    public int getID() {
        return ID;
    }

    public Pose3d getPose() {
        return pose;
    }

    @Override
    public String toString() {
        return "AprilTag[ID=" + ID + ", pose=" + pose + "]";
    }

    @Override
    public int hashCode() {
        return this.ID;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null || !(obj instanceof Appendable)) {
            return false;
        }
        AprilTag other = (AprilTag)obj;
        return this.ID == other.ID;
    }
}
