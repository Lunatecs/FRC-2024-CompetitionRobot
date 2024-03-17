// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.TargetingUtil;

public class AutoRangePIDCommand extends Command {

  BottomWristSubsystem bottomWrist;
  LimelightSubsystem limelight;
  LEDSubsystem ledSubsystem;
  ShooterSubsystem shooter;
  PIDController pid;
  TargetingUtil targeting;
  

  /** Creates a new AutoRangePIDCommand. */
  public AutoRangePIDCommand(BottomWristSubsystem bottomWrist, LimelightSubsystem limelight, LEDSubsystem ledSubsystem, ShooterSubsystem shooter) {
    this.bottomWrist=bottomWrist;
    this.limelight=limelight;
    this.ledSubsystem = ledSubsystem;
    this.shooter = shooter;
    pid = new PIDController(10, 0, 0);
    targeting = new TargetingUtil(limelight);
    addRequirements(bottomWrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double y = limelight.GetTy();
   //double preSetpoint = .000054 * y -.0568;
   //double preSetpoint = .002419 * y - .0778;
   double preSetpoint = 0.0019 * y - 0.07915;//0.09;//0.0683;
   
   double setpoint = MathUtil.clamp(preSetpoint, -.09, 0);

   
   double speed = pid.calculate(bottomWrist.getEncoder()-setpoint);

   SmartDashboard.putString("AUTO:Y error set speed", y + " " + (bottomWrist.getEncoder()-setpoint) + " " + setpoint + " " + speed);


   if(Math.abs(speed) > .5) {
    speed = Math.signum(speed) * .5;
   }


   bottomWrist.setSpeed(speed);

   targeting.calculateRotation();

   if(targeting.onTarget() && pid.atSetpoint()) {
    ledSubsystem.set(ledSubsystem.GREEN);
   } else if(!shooter.getSensor()) {
    ledSubsystem.set(ledSubsystem.BLUE);
   } else {
    ledSubsystem.set(ledSubsystem.GOLDEN);
   }

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
