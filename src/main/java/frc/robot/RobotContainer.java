// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Defaults;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NarwhalConstants;
import frc.robot.Constants.OceanViewConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.network.TCPSender;
import frc.robot.network.UDPReceiver;
import frc.robot.commands.XboxManualDriveCommand;
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
import frc.robot.util.field.AlgaeRemovalPosition;
import frc.robot.util.field.CoralStationSide;
import frc.robot.util.field.ReefScoringLocation;
import frc.robot.util.swerve.DrivingMotorType;
import frc.robot.util.upper_assembly.ScoringHeight;
import frc.robot.util.upper_assembly.UpperAssemblyFactory;
import frc.robot.util.upper_assembly.UpperAssemblyType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private VisionOdometry visionOdometry = new VisionOdometry(driveSubsystem.getSwerveDrivePoseEstimator()); // TODO: Add logic to add cameras to adjust odometry. visionOdometry.addCamera(PhotonVisionCamera camera);

    @SuppressWarnings("unused") // The class is used due to how WPILib treats and stores subsystems.
    private OceanViewManager oceanViewManager;

    // Smart Dashboard Inputs
    private final SendableChooser<AutonomousRoutine> autonomousSelector = new SendableChooser<>();
    private final SendableChooser<StartingPosition> positionSelector = new SendableChooser<>();
    private final SendableChooser<Alliance> allianceSelector = new SendableChooser<>();

    private final SendableChooser<UpperAssemblyType> upperAssemblySelector = new SendableChooser<>();
    private final SendableChooser<DrivingMotorType> drivingMotorSelector = new SendableChooser<>();

    Alliance selectedAlliance;

    /**
     * Creates a new RobotContainer object and sets up SmartDashboard an the button inputs.
     */
    public RobotContainer() {
        
        // Setup OceanView & all of its networking dependencies.
        setupOceanViewManager();

        // Add cameras to the VisionOdometry object.
        visionOdometry.addCamera(new PhotonVisionCamera(VisionConstants.FRONT_CAMERA_NAME, new Transform3d(-0.2159, 0, 0, new Rotation3d(0, 0, -Math.PI))));
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
            System.out.println("Successfully created TCPSender and UDPReceiver object!");
        } catch (Exception e) {
            System.err.println("Failed to construct TCPSender object: " + e.getMessage());
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
        EnumPrettifier.setupSendableChooserFromEnum(this.autonomousSelector, AutonomousRoutine.class, AutonomousRoutine.DEFAULT);
        EnumPrettifier.setupSendableChooserFromEnum(this.positionSelector, StartingPosition.class, StartingPosition.LEFT);
        EnumPrettifier.setupSendableChooserFromEnum(this.allianceSelector, Alliance.class, Alliance.RED);
        EnumPrettifier.setupSendableChooserFromEnum(this.upperAssemblySelector, UpperAssemblyType.class, UpperAssemblyType.NARWHAL);
        EnumPrettifier.setupSendableChooserFromEnum(this.drivingMotorSelector, DrivingMotorType.class, DrivingMotorType.KRAKEN_X60);

        // Add the selectors to the dashboard.
        SmartDashboard.putData(this.autonomousSelector);
        SmartDashboard.putData(this.positionSelector);
        SmartDashboard.putData(this.allianceSelector);
        SmartDashboard.putData(this.upperAssemblySelector);
        SmartDashboard.putData(this.drivingMotorSelector);

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
     * sets the upper assembly to the given type
     * 
     * @param upperAssemblyType the type of upper assembly to set to
     */
    public void setUpperAssembly(UpperAssemblyType upperAssemblyType) {
        upperAssembly = UpperAssemblyFactory.createUpperAssembly(upperAssemblyType);
    }

    public void scheduleAutonomous() {

        AutonomousRoutine selectedAutonomousRoutine = autonomousSelector.getSelected();
        Alliance selectedAlliance = allianceSelector.getSelected();
        SmartDashboard.putString("Selected Autonomous", selectedAutonomousRoutine.toString());
        SmartDashboard.putString("Selected Alliance", selectedAlliance.toString());

        driveSubsystem.setConfigs();

        //driveSubsystem.setRobotPose(new Pose2d(10.303, 2.2244, new Rotation2d(Math.PI)));
        //driveSubsystem.setRobotPose(new Pose2d(10.303, 3.85, new Rotation2d(Math.PI)));
        
        driveSubsystem.setRobotPose(new Pose2d(17.014, 1.354, new Rotation2d(-0.942478)));
        driveSubsystem.setUpperAssembly(Defaults.DEFAULT_UPPER_ASSEMBLY);

        upperAssembly.setRobotInScoringPositionSupplier(driveSubsystem::getInScorePose);
        upperAssembly.setRobotInIntakingPositionSupplier(driveSubsystem::getInIntakePose);
        if (upperAssembly instanceof NarwhalUpperAssembly) {
            ((NarwhalUpperAssembly)upperAssembly).setCanRaiseLiftSupplier(driveSubsystem::getNarwhalCanRaiseLift);
        }

        //Command driveCommand1 = driveSubsystem.getPathfindingCommand(new Pose2d(13.501195907592773, 1.4964193105697632 , new Rotation2d(-0.942478) ));
        Command driveCommand1 = driveSubsystem.getScoringCommand(Alliance.RED,ReefScoringLocation.J);
        Command driveCommand2 = driveSubsystem.getAlgaeRemovalCommand(Alliance.RED,AlgaeRemovalPosition.GH);
        Command driveCommand3 = driveSubsystem.getIntakeCommand(Alliance.RED,CoralStationSide.LEFT,7);
        Command driveCommand4 = driveSubsystem.getScoringCommand(Alliance.RED,ReefScoringLocation.J);
        Command upperAssemblyCommand1 = upperAssembly.getCoralScoreCommand(ScoringHeight.L4);
        Command upperAssemblyCommand2 = upperAssembly.getCoralIntakeCommand();

        SmartDashboard.putBoolean("test",false);
        driveCommand1.schedule();
        upperAssemblyCommand1.schedule();
        //driveCommand1.andThen(driveCommand2).andThen(driveCommand3).andThen(driveCommand4).schedule();
        //driveCommand1.schedule();


    }

    /**
     * Schedules commands used exclusively during TeleOp.
     */
    public void scheduleTeleOp() {
        // The Drive Command
        driveSubsystem.setConfigs();
        upperAssembly.setRobotInScoringPositionSupplier(driveSubsystem::getInScorePose);
        if (upperAssembly instanceof NarwhalUpperAssembly) {
            ((NarwhalUpperAssembly)upperAssembly).setCanRaiseLiftSupplier(driveSubsystem::getNarwhalCanRaiseLift);
        }
        upperAssembly.setDefaultCommand(upperAssembly.getManualCommand(xBoxController));
        driveSubsystem.setDefaultCommand(driveSubsystem.getManualCommand(xBoxController, Alliance.RED));

    }
}
