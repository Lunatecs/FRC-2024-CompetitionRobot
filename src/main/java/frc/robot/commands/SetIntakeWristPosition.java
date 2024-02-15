// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetIntakeWristPosition extends Command {
  /** Creates a new SetBottomWristPosition. */
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  PIDController intakeWristPid; 
  double setpoint;
  boolean outtake;
  public SetIntakeWristPosition(double kp, double ki, double kd, double tolerance, double setpoint, boolean outtake, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeWristPid = new PIDController(kp, ki, kd);
    intakeWristPid.setTolerance(tolerance);
    intakeWristPid.setSetpoint(setpoint);
    this.intake = intake;
    this.setpoint = setpoint;
    this.outtake = outtake;
    addRequirements(intake);
  }

  public SetIntakeWristPosition(double kp, double ki, double kd, double tolerance, double setpoint, IntakeSubsystem intake) {
    this(kp, ki, kd, tolerance, setpoint, false, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double speed = intakeWristPid.calculate(intake.getWristEncoder());
      if (Math.abs(speed)> 0.25){
        speed = speed/Math.abs(speed)*.25;
      }
    intake.setWristSpeed(speed);
    if(outtake) {
      intake.setIntakeSpeed(-.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setWristSpeed(0);
    intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeWristPid.atSetpoint();
  }
}
