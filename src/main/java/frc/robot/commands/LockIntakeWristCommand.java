// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.SetPointSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LockIntakeWristCommand extends PIDCommand {
  /** Creates a new LockIntakeWristCommand. */
  IntakeSubsystem intake;
  SetPointSupplier setpoint;

  public LockIntakeWristCommand(SetPointSupplier setPointSupplier, IntakeSubsystem intake) {
    super(
        // The controller that the command will use
        new PIDController(0.4, 0, 0),
        // This should return the measurement
        () -> intake.getWristEncoder(),
        // This should return the setpoint (can also be a constant)
        setPointSupplier,
        // This uses the output
        output -> {
          intake.setWristSpeed(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(intake);
    this.setpoint = setPointSupplier; 
    this.intake = intake; 
  }


  @Override
  public void initialize(){
    setpoint.setSetPoint(intake.getWristEncoder());
    SmartDashboard.putNumber("Set Point", this.setpoint.getAsDouble());
    super.initialize();
  }

  @Override
  public void execute(){
    super.execute();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.setWristSpeed(0);
  }

}
