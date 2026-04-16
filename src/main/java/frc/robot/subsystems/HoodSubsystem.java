// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private TalonFX motor = new TalonFX(Constants.HoodConstants.HoodID);
  public HoodSubsystem() {}
  public void setSpeed(double speed){
    motor.set(speed);
  }
  public double getHoodPosition(){
    return motor.getPosition().getValueAsDouble();
  }
  public double hoodAngle(){
    return getHoodPosition()*Constants.HoodConstants.tickToDegRatio;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("HoodAngle", hoodAngle());
    // This method will be called once per scheduler run
  }
}
