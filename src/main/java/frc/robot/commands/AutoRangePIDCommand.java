// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    pid = new PIDController(10, 0, 0);
    addRequirements(bottomWrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double y = limelight.GetTy();
   double preSetpoint = .000054 * y -.0568;
   double setpoint = MathUtil.clamp(preSetpoint, -.09, 0);

   pid.setSetpoint(setpoint);
   double speed = pid.calculate(bottomWrist.getEncoder());

   SmartDashboard.putString("AUTO:Y pre set speed", y + " " + preSetpoint + " " + setpoint + " " + speed);


   if(Math.abs(speed) > .5) {
    speed = Math.signum(speed) * .5;
   }


   bottomWrist.setSpeed(speed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bottomWrist.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
