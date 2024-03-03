// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends Command {
  /** Creates a new ShootNote. */
  private ShooterSubsystem shooter;
  private boolean isFinished;
  private int velocity;
  public RunShooterCommand(ShooterSubsystem shooter, int velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.velocity = velocity;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.setshooterSpeed(.9);

    shooter.setRPM(velocity);
  
    if (shooter.getVelocity() >= (velocity-4)) {
      isFinished = true; //was true
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
