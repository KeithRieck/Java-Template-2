// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.CommandJoystick;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

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

    // Dashboard chooser for autonomous command
    private final Selectable<Command> autoChooser;

    private static RobotContainer instance;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        driveSubsystem = new DriveSubsystem();

        autoChooser = configureAutonomous();

        configureBindings();

        configureDashboard();
        if (Robot.isSimulation()) {
            configureSimulation();
        }
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

    private void configureSimulation() {
        // Start the robot in an orientation that will make it easier to drive in
        // simulation.
        driveSubsystem.resetOdometry(new Pose2d(2, 2, Rotation2d.fromDegrees(90)));

        // Y button on xbox controller resets the gyro
        xboxController.y().onTrue(
                new InstantCommand(driveSubsystem::zeroHeading));

        driveSubsystem.setDefaultCommand(new DriveCommand(
                xboxController::getLeftY,
                xboxController::getLeftX,
                xboxController::getRightX,
                driveSubsystem));
    }

    /**
     * Use this method to set up all autonomous routines and add them to a
     * dashboard chooser. For PathPlanner, use this method to register
     * NamedCommands and EventTriggers.
     *
     * @return the autonomous chooser.
     */
    private Selectable<Command> configureAutonomous() {
        // TODO: register NamedCommands here

        Selectable<Command> chooser = new Selectable<>();

        // TODO: Add pathplanner autos here.

        // TODO: configure additional autonomous routines here
        return chooser;
    }

    /**
     * Use this method for additional configuration to the driver's dashboard.
     * SmartDashboard data should be added from subsystems and commands, but
     * this method defines the dashboard widgets.
     */
    private void configureDashboard() {
        Tunables.publish("Auto Chooser", autoChooser);

        // TODO: Add other Elastic dashboard widgets here.
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
