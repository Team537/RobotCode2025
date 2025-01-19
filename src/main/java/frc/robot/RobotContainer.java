// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.utils.Alliance;
import frc.utils.AutonomousRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController xBoxController = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    Command manualDriveCommand = new RunCommand(
            () -> {
                driveSubsystem.driveFromXBoxController(xBoxController);
            },
            driveSubsystem);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);

    // Smart Dashboard Inputs
    private final SendableChooser<AutonomousRoutine> autonomousSelector = new SendableChooser<>();
    private final SendableChooser<Alliance> allianceSelector = new SendableChooser<>();

    /**
     * Creates a new RobotContainer object and sets up SmartDashboard an the button inputs.
     */
    public RobotContainer() {

        // Setup Dashboard
        setupSmartDashboard();

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Sets up SmartDashboard so that important settings can be configured and key values can be viewed.
     */
    private void setupSmartDashboard() {

        // Setup Autonomous Routine Selection
        autonomousSelector.setDefaultOption("LEFT_HIGH_SCORE", AutonomousRoutine.LEFT_HIGH_SCORE);
        for (AutonomousRoutine autonomousRoutine : AutonomousRoutine.values()) {
            autonomousSelector.addOption(autonomousRoutine.toString(), autonomousRoutine);
        }
        
        // Setup Alliance Selection
        allianceSelector.setDefaultOption("RED", Alliance.RED);
        for (Alliance alliance : Alliance.values()) {
            allianceSelector.addOption(alliance.toString(), alliance);
        }

        // Add the selectors to the dashboard.
        SmartDashboard.putData(allianceSelector);
        SmartDashboard.putData(autonomousSelector);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed, cancelling on release.
        // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     */
    public Command getAutonomousCommand() {

        // Get and display the currently selected autonomous routine.
        AutonomousRoutine selectedAutonomousRoutine = autonomousSelector.getSelected();
        Alliance selectedAlliance = allianceSelector.getSelected();
        SmartDashboard.putString("Selected Autonomous", selectedAutonomousRoutine.toString());
        SmartDashboard.putString("Selected Alliance", selectedAlliance.toString());

        // An example command will be run in autonomous
        return Autos.exampleAuto(exampleSubsystem);
    }

    /**
     * Schedules commands used exclusively during TeleOp.
     */
    public void scheduleTeleOp() {

        // The Drive Command
        driveSubsystem.setDefaultCommand(manualDriveCommand);
    }
}