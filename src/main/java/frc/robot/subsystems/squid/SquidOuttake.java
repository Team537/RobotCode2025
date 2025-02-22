// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.squid;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SquidOuttake extends SubsystemBase {

  public static final SparkMax spark1 = new SparkMax(11, MotorType.kBrushless);
  public static final SparkMax spark2 = new SparkMax(12, MotorType.kBrushless);
  

  /** Creates a new SquidOuttake. */
  public SquidOuttake() {
  }

  public void OuttakeOut() {
    spark1.set(0.1);
    // spark2.set(-0.1);
  }

  public void OuttakeIn() {
    spark1.set(-0.1);
    // spark2.set(0.1);
  }

  public void OuttakeStop() {
    spark1.set(0);
    spark2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
