// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.CommandJoystick;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.Trigger;

import first.robot.Constants.OperatorConstants;
import first.robot.commands.DriveCommand;
import first.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final DriveSubsystem driveSubsystem;

    private final CommandJoystick leftJoystick = new CommandJoystick(OperatorConstants.LEFT_JOYSTICK_PORT);
    private final CommandJoystick rightJoystick = new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT);
    private final CommandXboxController xboxController = new CommandXboxController(
            OperatorConstants.XBOX_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveSubsystem = new DriveSubsystem();

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in
     * {@link org.wpilib.command2.button.CommandGenericHID}'s subclasses for
     * {@link CommandGamepad Gamepad} gamepads or
     * {@link org.wpilib.command2.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        driveSubsystem.setDefaultCommand(new DriveCommand(
                rightJoystick::getY,
                rightJoystick::getX,
                leftJoystick::getX,
                driveSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return null;
    }
}
