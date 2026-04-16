// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodPIDCommand extends Command {
  /** Creates a new HoodPIDCommand. */
  public HoodSubsystem subsystem;
  public double targetAngle;
  public PIDController PID = new PIDController(Constants.HoodConstants.kP,Constants.HoodConstants.kI,Constants.HoodConstants.kD);
  
  public HoodPIDCommand(HoodSubsystem subsystem , double targetAngle) {
    this.subsystem = subsystem;
    this.targetAngle = targetAngle;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  public void setpoint(){
    PID.setSetpoint(targetAngle);
  }
  
  @Override
  public void initialize() {
    PID.setTolerance(Constants.HoodConstants.tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        subsystem.setSpeed(PID.calculate(subsystem.hoodAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PID.atSetpoint();
  }
}
