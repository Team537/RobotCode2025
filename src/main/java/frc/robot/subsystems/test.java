// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class test extends SubsystemBase {
  /** Creates a new test. */

  public static SparkMax spark1 = new SparkMax(1, MotorType.kBrushless);
  public static SparkMax spark2 = new SparkMax(2, MotorType.kBrushless);
  public static SparkMax spark3 = new SparkMax(4, MotorType.kBrushless);
  public static SparkMax spark4 = new SparkMax(15, MotorType.kBrushless);
  
  public test() {}

  public void spark1move1() {
    spark1.set(0.25);
  }
  public void spark1move2() {
    spark1.set(-0.25);
  }
  public void spark1stop() {
    spark1.set(0);
  }


  public void spark2move1() {
    spark2.set(0.25);
  }
  public void spark2move2() {
    spark2.set(-0.25);
  }
  public void spark2stop() {
    spark2.set(0);
  }


  public void spark3move1() {
    spark3.set(0.1);
  }
  public void spark3move2() {
    spark3.set(-0.1);
  }
  public void spark3stop() {
    spark3.set(0);
  }
  
  public void spark4move1() {
    spark4.set(0.1);
  }
  public void spark4move2() {
    spark4.set(-0.1);
  }
  public void spark4stop() {
    spark4.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
