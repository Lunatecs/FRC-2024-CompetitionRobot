// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoRangeCommand extends Command {
  /** Creates a new AutoRangeCommand. */
  LimelightSubsystem limelightSubsystem;
  BottomWristSubsystem bottomWristSubsystem;
  public AutoRangeCommand(LimelightSubsystem limelightSubsystem, BottomWristSubsystem bottomWristSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.bottomWristSubsystem = bottomWristSubsystem;
    addRequirements(limelightSubsystem, bottomWristSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double limelightY = limelightSubsystem.GetTy();
    double preClamp = 0.56 * limelightY-20.15;//-20.15; //-19.65  //-19. //-19.5
    double setpoint = MathUtil.clamp(preClamp, -26, -2);

    bottomWristSubsystem.setPosition(setpoint);
    SmartDashboard.putNumber("Auto Range Setpoint", setpoint);
    SmartDashboard.putNumber("Pre Clamped Range", preClamp);
    SmartDashboard.putNumber("Limelight Y Range", limelightY);
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
