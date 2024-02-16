// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BottomWristSubsystem;

public class CheckPivotCommand extends Command {
  /** Creates a new CheckPivotCommand. */
  BottomWristSubsystem bottomWristSubsystem;
  double position;
  public CheckPivotCommand(BottomWristSubsystem bottomWristSubsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.bottomWristSubsystem = bottomWristSubsystem;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(bottomWristSubsystem.getEncoder()) > Math.abs(position);
  }
}
