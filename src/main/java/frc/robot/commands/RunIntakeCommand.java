// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunIntakeCommand extends Command {
  /** Creates a new RunIntakeCommand. */
  private IntakeSubsystem intake;
  //private ShooterSubsystem shooter;
  private FeederSubsystem feeder; 
  private double feederSpeed;
  private double intakeSpeed;
  private boolean isFinished;
  private boolean outtake;

  public RunIntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, double feederSpeed, double intakeSpeed, boolean outtake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    //this.shooter = shooter;
    this.feederSpeed = feederSpeed;
    this.intakeSpeed = intakeSpeed;
    this.outtake = outtake;
    this.feeder = feeder; 

    addRequirements(intake);
    addRequirements(feeder);
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
    if (outtake) {
      intake.setIntakeSpeed(-intakeSpeed);
    } else {
      intake.setIntakeSpeed(intakeSpeed);
    }
    
    feeder.setSpeed(feederSpeed);
    SmartDashboard.putBoolean("prox sensor", feeder.getSensor());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
    feeder.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

