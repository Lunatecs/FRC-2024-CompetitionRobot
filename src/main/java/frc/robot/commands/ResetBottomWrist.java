// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BottomWristSubsystem;

public class ResetBottomWrist extends Command {
  /** Creates a new ResetBottomWrist. */
  BottomWristSubsystem bottomWrist;
  boolean finished = false;
  int count = 0;

  public ResetBottomWrist(BottomWristSubsystem bottomWrist) {
    this.bottomWrist = bottomWrist;
    addRequirements(bottomWrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    count=0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(count > 25) {
      this.bottomWrist.initializeEncoderToZero();
      finished = true;
    } else if(count > 0) {
      //do nothing
      count++;
    } else if(this.bottomWrist.getCurrent() > 2.0) {
      count++;
      this.bottomWrist.setSpeed(0);
    } else {
      this.bottomWrist.setSpeed(.1);
    }

    SmartDashboard.putNumber("Count", count);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bottomWrist.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
