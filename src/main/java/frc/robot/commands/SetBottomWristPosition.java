// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BottomWristSubsystem;

public class SetBottomWristPosition extends Command {
  /** Creates a new SetBottomWristPosition. */
  BottomWristSubsystem bottomWrist = new BottomWristSubsystem(); 
  PIDController bottomWristPid; 
  public SetBottomWristPosition(double kp, double ki, double kd, double tolerance, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    bottomWristPid = new PIDController(kp, ki, kd);
    bottomWristPid.setTolerance(tolerance);
    bottomWristPid.setSetpoint(setpoint);
    addRequirements(bottomWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (bottomWristPid.calculate(bottomWrist.getEncoder()) > 0.25){
      bottomWrist.setSpeed(0.25);
    }
    bottomWrist.setSpeed(bottomWristPid.calculate(bottomWrist.getEncoder()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
