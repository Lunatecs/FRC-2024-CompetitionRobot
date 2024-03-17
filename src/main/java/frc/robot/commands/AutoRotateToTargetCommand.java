// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.TargetingUtil;
import frc.robot.subsystems.Swerve;

public class AutoRotateToTargetCommand extends Command {
  /** Creates a new AutoRotateToTargetCommand. */
  Swerve swerveDrive;
  LimelightSubsystem limelightSubsystem;
  double rotationVal;
  double rotMulti;
  TargetingUtil targetingUtil;
  public AutoRotateToTargetCommand(Swerve swerveDrive, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrive = swerveDrive;
    targetingUtil = new TargetingUtil(limelightSubsystem, 1.00, 0.0275);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationVal = targetingUtil.calculateRotation();
    rotMulti = Constants.Swerve.maxAngularVelocity;

    swerveDrive.drive(
            new Translation2d(0.0, 0.0).times(0.0), 
            rotationVal * rotMulti, 
            false, 
            true
        );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(
            new Translation2d(0.0, 0.0).times(0.0), 
            0.0, 
            false, 
            true
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("AutoOnTarget", targetingUtil.onTarget());
    return targetingUtil.onTarget();
  }
}
