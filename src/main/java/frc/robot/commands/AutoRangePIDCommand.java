// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoRangePIDCommand extends Command {

  BottomWristSubsystem bottomWrist;
  LimelightSubsystem limelight;
  PIDController pid;
  /** Creates a new AutoRangePIDCommand. */
  public AutoRangePIDCommand(BottomWristSubsystem bottomWrist, LimelightSubsystem limelight) {
    this.bottomWrist=bottomWrist;
    this.limelight=limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // double setpoint


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
