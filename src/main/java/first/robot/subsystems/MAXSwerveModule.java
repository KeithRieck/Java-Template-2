// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import first.robot.Constants.MAXSwerveModuleConfig;
import first.robot.Constants.DriveConstants;

public class MAXSwerveModule {
    private final SparkMax drivingSpark;
    private final SparkMax turningSpark;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private double chassisAngularOffset = 0;
    private SwerveModuleVelocity desiredState = new SwerveModuleVelocity(0.0, new Rotation2d());

    // Simulation components

    private SparkAbsoluteEncoderSim simTurningEncoder;
    private SparkRelativeEncoderSim simDrivingEncoder;

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        drivingSpark = new SparkMax(DriveConstants.DRIVE_CAN_BUS.value, drivingCANId, MotorType.kBrushless);
        turningSpark = new SparkMax(DriveConstants.DRIVE_CAN_BUS.value, turningCANId, MotorType.kBrushless);

        drivingEncoder = drivingSpark.getEncoder();
        turningEncoder = turningSpark.getAbsoluteEncoder();

        drivingClosedLoopController = drivingSpark.getClosedLoopController();
        turningClosedLoopController = turningSpark.getClosedLoopController();

        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        drivingSpark.configure(MAXSwerveModuleConfig.DRIVING_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        turningSpark.configure(MAXSwerveModuleConfig.TURNING_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        this.chassisAngularOffset = chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition().get());
        drivingEncoder.setPosition(0);

        if (RobotBase.isSimulation()) {
            simTurningEncoder = new SparkAbsoluteEncoderSim(turningSpark);
            simDrivingEncoder = new SparkRelativeEncoderSim(drivingSpark);
        }

    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleVelocity getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        if (RobotBase.isReal()) {
            return new SwerveModuleVelocity(drivingEncoder.getVelocity().get(),
                    new Rotation2d(turningEncoder.getPosition().get() - chassisAngularOffset));
        } else {
            return new SwerveModuleVelocity(simDrivingEncoder.getVelocity(),
                    new Rotation2d(simTurningEncoder.getPosition() - chassisAngularOffset));
        }
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        if (RobotBase.isReal()) {
            return new SwerveModulePosition(
                    drivingEncoder.getPosition().get(),
                    new Rotation2d(turningEncoder.getPosition().get() - chassisAngularOffset));
        } else {
            return new SwerveModulePosition(
                    simDrivingEncoder.getPosition(),
                    new Rotation2d(simTurningEncoder.getPosition() - chassisAngularOffset));
        }
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleVelocity desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleVelocity correctedDesiredState = new SwerveModuleVelocity();
        correctedDesiredState.velocity = desiredState.velocity;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState = correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition().get()));

        // Command driving and turning SPARKS towards their respective setpoints.
        drivingClosedLoopController.setSetpoint(correctedDesiredState.velocity, ControlType.kVelocity);
        turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        this.desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
        if (RobotBase.isSimulation()) {
            simDrivingEncoder.setPosition(0);
        }
    }

    // The following function must be called from simulationPeriodic()
    public void updateSimEncoders() {
        simDrivingEncoder.setVelocity(drivingClosedLoopController.getSetpoint().get());
        simDrivingEncoder.iterate(drivingClosedLoopController.getSetpoint().get(), 0.02);
        simTurningEncoder.setPosition(turningClosedLoopController.getSetpoint().get());
    }
}
