// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunIntakeFromShooterCommand extends Command {
  private ShooterSubsystem shooter;
  private FeederSubsystem feeder; 
  private double feederSpeed;
  private boolean isFinished;
  private double shooterSpeed;

  public RunIntakeFromShooterCommand(ShooterSubsystem shooter, double feederSpeed, double shooterSpeed, FeederSubsystem feeder) {
    this.shooter = shooter;
    this.feederSpeed = feederSpeed;
    this.shooterSpeed = shooterSpeed;
    this.feeder = feeder; 

    addRequirements(shooter, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feederSpeed = this.feederSpeed;
    if (!feeder.getSensor()) {
      feederSpeed = 0;
      isFinished = true;
    }


    shooter.setshooterSpeed(shooterSpeed);
    feeder.setSpeed(feederSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setSpeed(0);
    shooter.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
