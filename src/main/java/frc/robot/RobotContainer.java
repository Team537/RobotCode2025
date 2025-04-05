// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NarwhalConstants;
import frc.robot.Constants.OceanViewConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.network.TCPSender;
import frc.robot.network.UDPReceiver;
import frc.robot.routines.CenterScoreRoutine;
import frc.robot.routines.MultiScoreRoutine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.narwhal.NarwhalUpperAssembly;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.subsystems.vision.OceanViewManager;
import frc.robot.subsystems.vision.odometry.PhotonVisionCamera;
import frc.robot.subsystems.vision.odometry.VisionOdometry;
import frc.robot.util.EnumPrettifier;
import frc.robot.util.autonomous.Alliance;
import frc.robot.util.autonomous.AutonomousRoutine;
import frc.robot.util.autonomous.StartingPosition;
import frc.robot.util.swerve.DrivingMotorType;
import frc.robot.util.upper_assembly.UpperAssemblyFactory;
import frc.robot.util.upper_assembly.UpperAssemblyType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
        
        // Add narwhal upper assembly configuration options.
        SmartDashboard.putNumber("Intake Angle", NarwhalConstants.NarwhalWristConstants.INTAKE_ANGLE.getDegrees());
        SmartDashboard.putNumber("Intake Height", NarwhalConstants.NarwhalElevatorConstants.INTAKE_ELEVATOR_HEIGHT_METERS);
        SmartDashboard.putNumber("L1 Angle", NarwhalConstants.NarwhalWristConstants.L1_OUTTAKE_ANGLE.getDegrees());
        SmartDashboard.putNumber("L1 Height", NarwhalConstants.NarwhalElevatorConstants.L1_ELEVATOR_HEIGHT);
        SmartDashboard.putNumber("L2 Angle", NarwhalConstants.NarwhalWristConstants.L2_OUTTAKE_ANGLE.getDegrees());
        SmartDashboard.putNumber("L2 Height", NarwhalConstants.NarwhalElevatorConstants.L2_ELEVATOR_HEIGHT);
        SmartDashboard.putNumber("L3 Angle", NarwhalConstants.NarwhalWristConstants.L3_OUTTAKE_ANGLE.getDegrees());
        SmartDashboard.putNumber("L3 Height", NarwhalConstants.NarwhalElevatorConstants.L3_ELEVATOR_HEIGHT);
        SmartDashboard.putNumber("L4 Angle", NarwhalConstants.NarwhalWristConstants.L4_OUTTAKE_ANGLE.getDegrees());
        SmartDashboard.putNumber("L4 Height", NarwhalConstants.NarwhalElevatorConstants.L4_ELEVATOR_HEIGHT);

        SmartDashboard.putNumber("Climb Rotations (degrees)", NarwhalConstants.NarwhalClimberConstants.CLIMB_WINCH_ROTATIONS.getDegrees());
        SmartDashboard.putNumber("Deploy Rotations (degrees)", NarwhalConstants.NarwhalClimberConstants.DEPLOYED_WINCH_ROTATIONS.getDegrees());

        // Add autonomous configuration options.
        SmartDashboard.putNumber("Auto Score Offset X", NarwhalConstants.SCORING_RELATIVE_TRANSFORM.getX());
        SmartDashboard.putNumber("Auto Score Offset Y", NarwhalConstants.SCORING_RELATIVE_TRANSFORM.getY());
        SmartDashboard.putNumber("Auto Score Offset Rot", NarwhalConstants.SCORING_RELATIVE_TRANSFORM.getRotation().getDegrees());

        SmartDashboard.putNumber("Kraken Kp", DriveConstants.KrakenX60Driving.KP);
        SmartDashboard.putNumber("Kraken Ki", DriveConstants.KrakenX60Driving.KI);
        SmartDashboard.putNumber("Kraken Kd", DriveConstants.KrakenX60Driving.KD);

        SmartDashboard.putNumber("Translational Threshold", DriveConstants.TRANSLATION_THRESHOLD);
        SmartDashboard.putNumber("Rotational Threshold", DriveConstants.ROTATION_THRESHOLD);

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
        this.setWristValuesFromSmartDashbaord();
        this.delayTimeSeconds = SmartDashboard.getNumber("Auto Delay", this.delayTimeSeconds);
        this.startWithTushPush = SmartDashboard.getBoolean("Tush Push Mode", this.startWithTushPush);

        double autoScoreOffsetX = SmartDashboard.getNumber("Auto Score Offset X", NarwhalConstants.SCORING_RELATIVE_TRANSFORM.getX());
        double autoScoreOffsetY = SmartDashboard.getNumber("Auto Score Offset Y", NarwhalConstants.SCORING_RELATIVE_TRANSFORM.getY());
        double autoScoreOffsetRot = SmartDashboard.getNumber("Auto Score Offset Rot", NarwhalConstants.SCORING_RELATIVE_TRANSFORM.getRotation().getDegrees());

        NarwhalConstants.SCORING_RELATIVE_TRANSFORM = new Transform2d(new Translation2d(autoScoreOffsetX, autoScoreOffsetY), Rotation2d.fromDegrees(autoScoreOffsetRot));

        // Set the kraken drive motor`s PID coefficients to the specified values.
        double driveKp = SmartDashboard.getNumber("Kraken Kp", DriveConstants.KrakenX60Driving.KP);
        double driveKi = SmartDashboard.getNumber("Kraken Ki", DriveConstants.KrakenX60Driving.KI);
        double driveKd = SmartDashboard.getNumber("Kraken Kd", DriveConstants.KrakenX60Driving.KD);

        this.driveSubsystem.setDriveMotorPIDCoefficients(driveKp, driveKi, driveKd);
        
        // Get and set the path follower`s PID coefficients.
        double followerKp = SmartDashboard.getNumber("Drive Kp", DriveConstants.KrakenX60Driving.KP);
        double followerKi = SmartDashboard.getNumber("Drive Ki", DriveConstants.KrakenX60Driving.KI);
        double followerKd = SmartDashboard.getNumber("Drive Kd", DriveConstants.KrakenX60Driving.KD);
  
        this.driveSubsystem.setFollowerPIDCoefficients(followerKp, followerKi, followerKd);

        // Set the thresholds for autonomous pathing.
        double translationalThreshold = SmartDashboard.getNumber("Translational Threshold", DriveConstants.TRANSLATION_THRESHOLD);
        double rotationalThreshold = SmartDashboard.getNumber("Translational Threshold", DriveConstants.TRANSLATION_THRESHOLD);

        this.driveSubsystem.setThresholds(translationalThreshold, rotationalThreshold);

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

        Pose2d startingPose;

        // Determine the starting position for the specified autonomous routine and alliance.
        switch (autonomousRoutine) {
            case LEFT:
                startingPose = StartingPosition.LEFT.getPose(alliance);
                break;
            case CENTER:
                startingPose = StartingPosition.CENTER.getPose(alliance);
                break;
            case RIGHT:
                startingPose = StartingPosition.RIGHT.getPose(alliance);
                break;
            default:
                System.err.println("[System]: No alliance starting position selected!");
                startingPose = null;
                break;
        }

        if (startWithTushPush) {
            startingPose.transformBy(FieldConstants.StartingPoseConstants.TUSH_PUSH_STARTING_TRANSFORM);
        }

        // If a valid starting pose was determined, set the robot pose.
        if (startingPose != null) {
            driveSubsystem.setRobotPose(startingPose);
        }

        // Construct the autonomous program for the selected starting position.
        Command autonomousCommand = new InstantCommand();

        if (startWithTushPush) {
            
            autonomousCommand = autonomousCommand.andThen(driveSubsystem.getDriveToPoseCommand(startingPose.transformBy(FieldConstants.StartingPoseConstants.TUSH_PUSH_TRANSFORM)));

        }
      
        switch (autonomousRoutine) {
            case LEFT:
            case RIGHT:
                autonomousCommand = autonomousCommand.andThen(MultiScoreRoutine.getCommand(autonomousRoutine == AutonomousRoutine.LEFT ? StartingPosition.LEFT : StartingPosition.RIGHT, alliance, driveSubsystem, upperAssembly));
                break;
            case CENTER:
                autonomousCommand = autonomousCommand.andThen(CenterScoreRoutine.getCommand(alliance, driveSubsystem, upperAssembly));
                break;
            default:
                System.err.println("[System]: No alliance starting position selected!");
                break;
        }

        // Wait if specified, otherwise just execute auto command
        if (this.delayTimeSeconds > 0) {
            Command autoDelayCommand = new WaitCommand(this.delayTimeSeconds);
            autoDelayCommand.andThen(autonomousCommand).schedule();
        }
        else {
            autonomousCommand.schedule();
        }
    }

    /**
     * Schedules commands used exclusively during TeleOp.
     */
    public void scheduleTeleOp() {
        CommandScheduler.getInstance().cancelAll();
        this.setWristValuesFromSmartDashbaord();
        NarwhalConstants.NarwhalClimberConstants.CLIMB_WINCH_ROTATIONS = Rotation2d.fromDegrees(SmartDashboard.getNumber("Climb Rotations (degrees)", NarwhalConstants.NarwhalClimberConstants.CLIMB_WINCH_ROTATIONS.getDegrees()));
        NarwhalConstants.NarwhalClimberConstants.DEPLOYED_WINCH_ROTATIONS = Rotation2d.fromDegrees(SmartDashboard.getNumber("Deploy Rotations (degrees)", NarwhalConstants.NarwhalClimberConstants.DEPLOYED_WINCH_ROTATIONS.getDegrees()));

        Alliance alliance = allianceSelector.getSelected();
        SmartDashboard.putString("Selected Alliance", alliance.toString());

        // The Drive Command
        driveSubsystem.setConfigs();
        upperAssembly.setDefaultCommand(upperAssembly.getManualCommand(xBoxController));
        driveSubsystem.setDefaultCommand(driveSubsystem.getManualCommand(xBoxController, alliance));
    }
    
    /**
     * Updates the robot's odometry. This calls the {@code DriveSubsystem}'s {@code updateOdometry()} method and 
     * the {@code VisionOdometry}'s {@code updateVisionPositionData()} method. 
     */
    public void updateOdometry() {
        this.driveSubsystem.updateOdometry();
        this.visionOdometry.updateVisionPositionData();
    }

    private void setWristValuesFromSmartDashbaord() {
        NarwhalConstants.NarwhalWristConstants.INTAKE_ANGLE = Rotation2d.fromDegrees(SmartDashboard.getNumber("Intake Angle", NarwhalConstants.NarwhalWristConstants.INTAKE_ANGLE.getDegrees()));
        NarwhalConstants.NarwhalElevatorConstants.INTAKE_ELEVATOR_HEIGHT_METERS = SmartDashboard.getNumber("Intake Height", NarwhalConstants.NarwhalElevatorConstants.INTAKE_ELEVATOR_HEIGHT_METERS);

        NarwhalConstants.NarwhalWristConstants.L1_OUTTAKE_ANGLE = Rotation2d.fromDegrees(SmartDashboard.getNumber("L1 Angle", NarwhalConstants.NarwhalWristConstants.L1_OUTTAKE_ANGLE.getDegrees()));
        NarwhalConstants.NarwhalElevatorConstants.L1_ELEVATOR_HEIGHT = SmartDashboard.getNumber("L1 Height", NarwhalConstants.NarwhalElevatorConstants.L1_ELEVATOR_HEIGHT);
        NarwhalConstants.NarwhalWristConstants.L2_OUTTAKE_ANGLE = Rotation2d.fromDegrees(SmartDashboard.getNumber("L2 Angle", NarwhalConstants.NarwhalWristConstants.L2_OUTTAKE_ANGLE.getDegrees()));
        NarwhalConstants.NarwhalElevatorConstants.L2_ELEVATOR_HEIGHT = SmartDashboard.getNumber("L2 Height", NarwhalConstants.NarwhalElevatorConstants.L2_ELEVATOR_HEIGHT);
        NarwhalConstants.NarwhalWristConstants.L3_OUTTAKE_ANGLE = Rotation2d.fromDegrees(SmartDashboard.getNumber("L3 Angle", NarwhalConstants.NarwhalWristConstants.L3_OUTTAKE_ANGLE.getDegrees()));
        NarwhalConstants.NarwhalElevatorConstants.L3_ELEVATOR_HEIGHT = SmartDashboard.getNumber("L3 Height", NarwhalConstants.NarwhalElevatorConstants.L3_ELEVATOR_HEIGHT);
        NarwhalConstants.NarwhalWristConstants.L4_OUTTAKE_ANGLE = Rotation2d.fromDegrees(SmartDashboard.getNumber("L4 Angle", NarwhalConstants.NarwhalWristConstants.L4_OUTTAKE_ANGLE.getDegrees()));
        NarwhalConstants.NarwhalElevatorConstants.L4_ELEVATOR_HEIGHT = SmartDashboard.getNumber("L4 Height", NarwhalConstants.NarwhalElevatorConstants.L4_ELEVATOR_HEIGHT);
    }
}
