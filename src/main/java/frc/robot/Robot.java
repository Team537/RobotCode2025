// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. 
 * If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    StringLogEntry frc537StringLog;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    public Robot() {

        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        // Update the robot's odometry faster than the main loop times. (called every 4ms, offset by 10ms to prevent
        // it from being called at the same time as the main loop).
        addPeriodic(() -> {
            robotContainer.updateOdometry();
        }, 0.004, 0.01);

        // Automatically capture data with the driver camera.
        CameraServer.startAutomaticCapture();

        // Allow the camera stream to be viewed via the robots network.
        PortForwarder.add(5800, "photonvision.local", 5800);
        PortForwarder.add(5800, "photonvision2.local", 5800);
        PortForwarder.add(5000, "oceanview.local", 5000);

        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        frc537StringLog = new StringLogEntry(log, "/frc537/string");
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow 
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing finished or
        // interrupted commands, and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

        CommandScheduler.getInstance().cancelAll();
        robotContainer.scheduleAutonomous();

        frc537StringLog.append("Autonomous Init\n");

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        CommandScheduler.getInstance().cancelAll();

        // Schedule Teleop Commands
        robotContainer.scheduleTeleOp();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        
    }

    @Override
    public void testInit() {

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}