// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX motor = new TalonFX(Constants.intakeConstants.rollerID);
  public IntakeSubsystem() {}

  public void setSpeed(double speed){
    motor.set(speed);
    SmartDashboard.putNumber("IntakeSpeed", speed);
  } 
  // public Command coast() {
  //   return Commands.runEnd(() -> motor.setNeutralMode(NeutralModeValue.Coast), () -> motor.setNeutralMode(NeutralModeValue.Brake), this);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
