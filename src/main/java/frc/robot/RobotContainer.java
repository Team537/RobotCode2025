// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OceanViewConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.network.TCPSender;
import frc.robot.network.UDPReceiver;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.narwhal.NarwhalUpperAssembly;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.subsystems.vision.OceanViewManager;
import frc.robot.subsystems.vision.odometry.PhotonVisionCamera;
import frc.robot.subsystems.vision.odometry.VisionOdometry;
import frc.robot.util.EnumPrettifier;
import frc.robot.util.autonomous.Alliance;
import frc.robot.util.autonomous.AutonomousRoutine;
import frc.robot.util.autonomous.StartingPosition;
import frc.robot.util.field.CoralStationSide;
import frc.robot.util.field.ReefScoringLocation;
import frc.robot.util.swerve.DrivingMotorType;
import frc.robot.util.upper_assembly.ScoringHeight;
import frc.robot.util.upper_assembly.UpperAssemblyFactory;
import frc.robot.util.upper_assembly.UpperAssemblyType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should actually be handled 
 * in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController xBoxController = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // Networking
    private UDPReceiver udpReceiver;
    private TCPSender tcpSender;

    // Subsystems
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private DriveSubsystem driveSubsystem = new DriveSubsystem();

    private UpperAssemblyBase upperAssembly = UpperAssemblyFactory.createUpperAssembly(Constants.Defaults.DEFAULT_UPPER_ASSEMBLY);

    private VisionOdometry visionOdometry = new VisionOdometry(driveSubsystem.getSwerveDrivePoseEstimator());

    @SuppressWarnings("unused") // The class is used due to how WPILib treats and stores subsystems.
    private OceanViewManager oceanViewManager;

    // Smart Dashboard Inputs
    private final SendableChooser<AutonomousRoutine> autonomousSelector = new SendableChooser<>();
    private final SendableChooser<Alliance> allianceSelector = new SendableChooser<>();

    private final SendableChooser<UpperAssemblyType> upperAssemblySelector = new SendableChooser<>();
    private final SendableChooser<DrivingMotorType> drivingMotorSelector = new SendableChooser<>();

    private double delayTimeSeconds = 0;
    private boolean startWithTushPush = false;

    /**
     * Creates a new RobotContainer object and sets up SmartDashboard an the button inputs.
     */
    public RobotContainer() {
        
        // Setup OceanView & all of its networking dependencies.
        setupOceanViewManager();

        // Add cameras to the VisionOdometry object.
        visionOdometry.addCamera(new PhotonVisionCamera(VisionConstants.FRONT_CAMERA_NAME, VisionConstants.FRONT_CAMERA_OFFSET));
        visionOdometry.addCamera(new PhotonVisionCamera(VisionConstants.RIGHT_CAMERA_NAME, VisionConstants.RIGHT_CAMERA_OFFSET));
        visionOdometry.addCamera(new PhotonVisionCamera(VisionConstants.LEFT_CAMERA_NAME, VisionConstants.LEFT_CAMERA_OFFSET)); 

        // Setup Dashboard
        setupSmartDashboard();

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Sets up the OceanViewManager instance used by the robot. 
     * If the PI is not connected to the robot, nothing will happen.
     */
    private void setupOceanViewManager() {

        // Attempted to create a new TCPSender and UDPReceiver object.
        try {
            this.udpReceiver = new UDPReceiver(OceanViewConstants.UDP_PORT_NUMBER);    
            this.tcpSender = new TCPSender(OceanViewConstants.PI_IP, OceanViewConstants.TCP_PORT_NUMBER);
            System.out.println("[@@@ OceanView @@@]: --- Successfully created TCPSender and UDPReceiver object!");
        } catch (Exception e) {
            System.err.println("[@@@ OceanView @@@]: --- Failed to construct TCPSender object: " + e.getMessage());
        }

        // An OceanView manager instance cannot be created if either the TCPSender or UDPReceiver is null.
        // Thus, we stop setting up the OceanViewManager.
        if (this.udpReceiver == null || this.tcpSender == null) {
            return;
        }

        // Start the UDPReceiver.
        this.udpReceiver.start();

        // Create a new OceanViewManager object.
        this.oceanViewManager = new OceanViewManager(this.udpReceiver, this.tcpSender, driveSubsystem::getRobotPose);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s 
     * subclasses for {@link CommandXboxControllerXbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4ControllerPS4} 
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
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
     * This method sets up the dashboard so that the drivers can configure the robots settings.
     */
    private void setupSmartDashboard() {
        // Setup dropdowns from enumeration values
        EnumPrettifier.setupSendableChooserFromEnum(this.autonomousSelector, AutonomousRoutine.class, AutonomousRoutine.CENTER);
        EnumPrettifier.setupSendableChooserFromEnum(this.allianceSelector, Alliance.class, Alliance.RED);
        EnumPrettifier.setupSendableChooserFromEnum(this.upperAssemblySelector, UpperAssemblyType.class, UpperAssemblyType.NARWHAL);
        EnumPrettifier.setupSendableChooserFromEnum(this.drivingMotorSelector, DrivingMotorType.class, DrivingMotorType.KRAKEN_X60);

        // Add the selectors to the dashboard.
        SmartDashboard.putData(this.autonomousSelector);
        SmartDashboard.putData(this.allianceSelector);
        SmartDashboard.putData(this.upperAssemblySelector);
        SmartDashboard.putData(this.drivingMotorSelector);

        // Add autonomous configuration options.
        SmartDashboard.putNumber("Auto Delay", this.delayTimeSeconds);
        SmartDashboard.putBoolean("Tush Push Mode", this.startWithTushPush);
    }

    /**
     * Sets the upper assembly to the given type
     * 
     * @param upperAssemblyType the type of upper assembly to set to
     */
    public void setUpperAssembly(UpperAssemblyType upperAssemblyType) {
        upperAssembly = UpperAssemblyFactory.createUpperAssembly(upperAssemblyType);
    }

    /**
     * Creates and schedules the selected autonomous routine. 
     */
    public void scheduleAutonomous() {
        this.delayTimeSeconds = SmartDashboard.getNumber("Auto Delay", this.delayTimeSeconds);
        this.startWithTushPush = SmartDashboard.getBoolean("Tush Push Mode", this.startWithTushPush);


        // Get and display the selected autonomous mode.
        AutonomousRoutine autonomousRoutine = autonomousSelector.getSelected();
        Alliance alliance = allianceSelector.getSelected();

        // Update the robot`s subsystems to be configured for the selected autonomous routine.
        driveSubsystem.setConfigs();
        upperAssembly.setRobotInScoringPositionSupplier(driveSubsystem::getInScorePose);
        upperAssembly.setRobotInIntakingPositionSupplier(driveSubsystem::getInIntakePose);
        if (upperAssembly instanceof NarwhalUpperAssembly) {
            ((NarwhalUpperAssembly)upperAssembly).setCanRaiseLiftSupplier(driveSubsystem::getNarwhalCanRaiseLift);
        }

        // Get the starting position for the specified autonomous routine and alliance.
        switch (autonomousRoutine) {
            case LEFT:
                driveSubsystem.setRobotPose(StartingPosition.LEFT.getPose(alliance));
                break;
            case CENTER:
                driveSubsystem.setRobotPose(StartingPosition.CENTER.getPose(alliance));
                break;
            case RIGHT:
                driveSubsystem.setRobotPose(StartingPosition.RIGHT.getPose(alliance));
                break;
            default:
            System.err.println("[System]: No alliance starting position selected!");
                break;
        }
        
        // Get and create the time delay the driver wants the autonomous to run on.
        this.delayTimeSeconds = SmartDashboard.getNumber("Auto Delay", this.delayTimeSeconds);
        Command autoDelayCommand = new WaitCommand(this.delayTimeSeconds);

        // Construct the autonomous program for the selected starting position.
        Command autonomousCommand;
        switch (autonomousRoutine) {
            case LEFT:
                autonomousCommand = 
                    (
                        driveSubsystem.getScoringCommand(alliance, ReefScoringLocation.J)
                        .alongWith(upperAssembly.getCoralScoreCommand(ScoringHeight.L4))
                    ).andThen(
                        driveSubsystem.getIntakeCommand(alliance, CoralStationSide.LEFT, 2)
                        .andThen(upperAssembly.getCoralIntakeCommand())
                    ).andThen(
                        driveSubsystem.getScoringCommand(alliance, ReefScoringLocation.K)
                        .alongWith(upperAssembly.getCoralScoreCommand(ScoringHeight.L4))
                    ).andThen(
                        upperAssembly.getLowerCommand()
                    );  
                break;  
            case CENTER: 

                /**
                 * Drive forwards and score the preloaded coral onto the nearest branch at L4 height.
                 */
                autonomousCommand = 
                    (
                        driveSubsystem.getScoringCommand(alliance, ReefScoringLocation.H)
                        .alongWith(upperAssembly.getCoralScoreCommand(ScoringHeight.L4))
                    ).andThen(
                        upperAssembly.getLowerCommand()
                    );
                break;
            case RIGHT:
                autonomousCommand = 
                    (
                        driveSubsystem.getScoringCommand(alliance, ReefScoringLocation.E)
                        .alongWith(upperAssembly.getCoralScoreCommand(ScoringHeight.L4))
                    ).andThen(
                        driveSubsystem.getIntakeCommand(alliance, CoralStationSide.RIGHT, 7)
                        .andThen(upperAssembly.getCoralIntakeCommand())
                    ).andThen(
                        driveSubsystem.getScoringCommand(alliance, ReefScoringLocation.D)
                        .alongWith(upperAssembly.getCoralScoreCommand(ScoringHeight.L4))
                    ).andThen(
                        upperAssembly.getLowerCommand()
                    );    
                break;
            default:
                autonomousCommand =  new InstantCommand(); // Do nothing if no valid auto routine is selected
        }

        // Combine the autonomous delay and the main routine. Then schedule the command.
        autoDelayCommand.andThen(autonomousCommand).schedule();
    }

    /**
     * Schedules commands used exclusively during TeleOp.
     */
    public void scheduleTeleOp() {

        Alliance alliance = allianceSelector.getSelected();
        SmartDashboard.putString("Selected Alliance", alliance.toString());

        // The Drive Command
        driveSubsystem.setConfigs();
        upperAssembly.setDefaultCommand(upperAssembly.getManualCommand(xBoxController));
        driveSubsystem.setDefaultCommand(driveSubsystem.getManualCommand(xBoxController, alliance));
    }


    //////////////////////////////////////////////////////////////////////////////
    // Periodic Update Methods
    //////////////////////////////////////////////////////////////////////////////
    
    /**
     * Updates the robot's odometry. This calls the {@code DriveSubsystem}'s {@code updateOdometry()} method and 
     * the {@code VisionOdometry}'s {@code updateVisionPositionData()} method. 
     */
    public void updateOdometry() {
        this.driveSubsystem.updateOdometry();
        this.visionOdometry.updateVisionPositionData();
    }
}
