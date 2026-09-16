// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import java.util.Optional;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.tunable.Tunables;

import first.robot.Constants;
import first.robot.Constants.DriveConstants;
import first.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

    // Create MAXSwerveModules
    private final MAXSwerveModule frontLeftModule = new MAXSwerveModule(
            DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
            DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
            DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule frontRightModule = new MAXSwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
            DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
            DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearLeftModule = new MAXSwerveModule(
            DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
            DriveConstants.REAR_LEFT_TURNING_CAN_ID,
            DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearRightModule = new MAXSwerveModule(
            DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
            DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
            DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

    // The gyro sensor
    private final OnboardIMU gyro = new OnboardIMU(OnboardIMU.MountOrientation.FLAT);

    // Odometry class for tracking robot pose
    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d.ZERO,
            new SwerveModulePosition[] {
                    frontLeftModule.getPosition(),
                    frontRightModule.getPosition(),
                    rearLeftModule.getPosition(),
                    rearRightModule.getPosition()
            },
            DriveConstants.INITIAL_POSE_METERS,
            DriveConstants.STATE_STD_DEVS,
            DriveConstants.VISION_MEASUREMENT_STD_DEVS);

    /**
     * SmartDashboard widget for displaying gyro angle.
     */
    public final NetworkTableEntry gyroDisplay = NetworkTableInstance.getDefault().getEntry("DriveSubsystem/Gyro");

    /**
     * SmartDashboard widget for displaying the battery voltage.
     */
    public final NetworkTableEntry voltageDisplay = NetworkTableInstance.getDefault()
            .getEntry("PowerDistribution/Voltage");
    private final PowerDistribution powerDistribution = new PowerDistribution(
            DriveConstants.POWER_DISTRIBUTION_CANBUS);

    /**
     * SmartDashboard widget for displaying robot location.
     */
    public final Field2d odometryDisplay = new Field2d();

    /**
     * Logger for putting robot location into AdvantageScope.
     */
    private final StructPublisher<Pose2d> odometryLogger;

    private SwerveModuleVelocity[] swerveModuleVelocitys = { new SwerveModuleVelocity(), new SwerveModuleVelocity(),
            new SwerveModuleVelocity(), new SwerveModuleVelocity() };

    // Simulated Gryo
    private Rotation2d simulatedGyroAngle = Rotation2d.fromDegrees(0);
    private double simulatedGyroRate;

    public DriveSubsystem() {
        gyro.resetYaw();

        Tunables.publish("Odometry", odometryDisplay);
        odometryLogger = NetworkTableInstance.getDefault().getStructTopic("Odometry", Pose2d.struct).publish();
    }

    public boolean isRedAlliance() {
        Optional<Alliance> alliance = MatchState.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.RED;
        }
        return false;
    }

    @Override
    public void periodic() {
        final Pose2d updatedPose = odometry.update(
                Rotation2d.fromDegrees(getHeading()),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        rearLeftModule.getPosition(),
                        rearRightModule.getPosition()
                });

        gyroDisplay.setNumber((int) Math.round(getHeading()));
        voltageDisplay.setNumber(powerDistribution.getVoltage());
        odometryDisplay.setRobotPose(updatedPose);
        odometryLogger.set(getPose());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(
                Rotation2d.fromDegrees(getHeading()),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        rearLeftModule.getPosition(),
                        rearRightModule.getPosition()
                },
                pose2d);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        odometry.addVisionMeasurement(pose, timestamp);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double rotDelivered = rot * DriveConstants.MAX_ANGULAR_SPEED;

        ChassisVelocities robotRelativeChassisVelocity = fieldRelative
                ? fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
                        rotDelivered,
                        Rotation2d.fromDegrees(getHeading()))
                : new ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        SwerveModuleVelocity[] swerveModuleVelocity = DriveConstants.DRIVE_KINEMATICS
                .toSwerveModuleVelocities(robotRelativeChassisVelocity);
        swerveModuleVelocity = SwerveDriveKinematics.desaturateWheelVelocities(swerveModuleVelocity,
                DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(swerveModuleVelocity[0]);
        frontRightModule.setDesiredState(swerveModuleVelocity[1]);
        rearLeftModule.setDesiredState(swerveModuleVelocity[2]);
        rearRightModule.setDesiredState(swerveModuleVelocity[3]);

        this.swerveModuleVelocitys = swerveModuleVelocity;
    }

    private static ChassisVelocities fromFieldRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation2d robotAngle) {
        Translation2d rotatedVector = new Translation2d(vxMetersPerSecond, vyMetersPerSecond)
                .rotateBy(robotAngle.unaryMinus());
        return new ChassisVelocities(rotatedVector.getX(), rotatedVector.getY(), omegaRadiansPerSecond);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleVelocity[] desiredStates) {
        desiredStates = SwerveDriveKinematics.desaturateWheelVelocities(
                desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        rearLeftModule.setDesiredState(desiredStates[2]);
        rearRightModule.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        rearLeftModule.resetEncoders();
        rearRightModule.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.resetYaw();
        if (Robot.isSimulation()) {
            simulatedGyroAngle = Rotation2d.fromDegrees(0);
            simulatedGyroRate = 0;
            resetOdometry(getPose());
        }
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        if (RobotBase.isReal()) {
            return Units.radiansToDegrees(gyro.getAngleZ());
        } else {
            return simulatedGyroAngle.getDegrees();
        }
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return Units.radiansToDegrees(gyro.getGyroRateZ()) * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * @return chassis speeds of robot
     */
    public ChassisVelocities getChassisVelocities() {
        ChassisVelocities speeds = Constants.DriveConstants.DRIVE_KINEMATICS.toChassisVelocities(
                frontLeftModule.getState(),
                frontRightModule.getState(),
                rearLeftModule.getState(),
                rearRightModule.getState());

        return speeds;
    }

    // This can only be called from simulationPeriodic()
    private void updateGyro() {
        ChassisVelocities speeds = getChassisVelocities();
        simulatedGyroRate = speeds.omega;
        simulatedGyroAngle = simulatedGyroAngle.plus(Rotation2d.fromRadians(simulatedGyroRate * .02));
    }

    // This can only be called from simulationPeriodic()
    private void updateSimEncoders(MAXSwerveModule... swerveModules) {
        for (var swerveModule : swerveModules) {
            swerveModule.updateSimEncoders();
        }
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        updateSimEncoders(frontLeftModule, frontRightModule, rearLeftModule, rearRightModule);
        updateGyro();
    }
}
