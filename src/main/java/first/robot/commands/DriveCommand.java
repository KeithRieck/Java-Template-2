// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.math.util.MathUtil;

import first.robot.Constants.DriveConstants;
import first.robot.subsystems.DriveSubsystem;

/** Default command for driving the {@code DriveSubystem} using joysticks */
public class DriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardStick;
    private final DoubleSupplier sidewaysStick;
    private final DoubleSupplier rotStick;

    /**
     * Drive the robot using joysticks.
     * 
     * @param forwardStick  Joystick for forward translation.
     * @param sidewaysStick Joystick for sideways translation.
     * @param rotStick      Joystick axis for rotation.
     * @param rotSticky
     * @param drive         DriveSubsystem
     */
    public DriveCommand(DoubleSupplier forwardStick, DoubleSupplier sidewaysStick, DoubleSupplier rotStick,
            DriveSubsystem drive) {
        this.forwardStick = forwardStick;
        this.sidewaysStick = sidewaysStick;
        this.rotStick = rotStick;
        this.driveSubsystem = drive;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(forwardStick.getAsDouble() * Math.abs(forwardStick.getAsDouble()),
                DriveConstants.DRIVE_DEAD_BAND) * -1;
        double ySpeed = MathUtil.applyDeadband(sidewaysStick.getAsDouble() * Math.abs(sidewaysStick.getAsDouble()),
                DriveConstants.DRIVE_DEAD_BAND) * -1;
        double rot = MathUtil.applyDeadband(rotStick.getAsDouble() * Math.abs(rotStick.getAsDouble()),
                DriveConstants.DRIVE_DEAD_BAND) * -1;
        driveSubsystem.drive(xSpeed, ySpeed, rot, true);
    }
}
