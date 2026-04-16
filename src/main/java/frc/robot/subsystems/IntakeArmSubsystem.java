// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArmSubsystem extends SubsystemBase {
  private TalonFX IntakeArm = new TalonFX(Constants.IntakeArmConstants.MOTORID);
  /** Creates a new IntakeArmSubsystem. */
  public IntakeArmSubsystem() {}

   @Override
    public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle", getArmAngle());
   }
  
  public void setSpeed(double speed) {
    IntakeArm.set(speed);
  }
  
  public double getArmAngle() {
    return tickToDeg(IntakeArm.getPosition().getValueAsDouble());
  }

  private double tickToDeg(double tick) {
    return tick*Constants.IntakeArmConstants.TICK_TO_DEG_RATIO;
  }

  // private double degToTick(double deg) {
  //   return deg/Constants.IntakeArmConstants.TICK_TO_DEG_RATIO;
  // }
  
}
