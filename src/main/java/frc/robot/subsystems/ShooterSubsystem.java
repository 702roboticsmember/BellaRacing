// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private TalonFX MainMotor = new TalonFX(Constants.ShooterConstants.FlywheelMotor1ID);
  private TalonFX FollowerMotor = new TalonFX(Constants.ShooterConstants.FlywheelMotor2ID);
  public ShooterSubsystem() {
    FollowerMotor.setControl(new Follower(Constants.ShooterConstants.FlywheelMotor1ID, MotorAlignmentValue.Opposed));
  }

  public void setSpeed(double speed){
    MainMotor.set(speed);
  }

  public Command spin() {
    return Commands.runEnd(() -> setSpeed(Constants.ShooterConstants.MotorSpeed), () -> setSpeed(0), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
