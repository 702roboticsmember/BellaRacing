// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  String LimelightName = Constants.LimelightName;
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {

  }

  Pose2d getRobotPose() {
    
    //LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
    //This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
    //relative to april tag, the one that is currently looking at; can be on multitag mode for 
    return LimelightHelpers.getBotPose2d(LimelightName);
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //error
  }
}
