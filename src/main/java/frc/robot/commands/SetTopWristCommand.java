// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TopWristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTopWristCommand extends PIDCommand {

  private boolean end;

  /** Creates a new SetTopWristCommand. */
  public SetTopWristCommand(TopWristSubsystem topWrist, final double postion, boolean end) {
    super(
        // The controller that the command will use
        new PIDController(1.5, 0, 0), //2.25
        // This should return the measurement
        () -> topWrist.getEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> postion,
        // This uses the output
        output -> {
          // Use the output here
          if(Math.abs(output) > .6) {
            output = Math.signum(output) * .6;
          }

          topWrist.setSpeed(output);

        });
        this.end = end;
        addRequirements(topWrist);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    super.initialize();
    if(end) {
      this.m_controller.setTolerance(0.01);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(end) {
      return this.m_controller.atSetpoint();
    } else {
      return false;
    }
  }
}
