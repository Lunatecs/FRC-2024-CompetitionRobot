// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.BottomWristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetBottomWristCommand extends PIDCommand {
  /** Creates a new SetBottomWristCommand. */

  private boolean end;

  public SetBottomWristCommand(BottomWristSubsystem bottomWrist, final double position, boolean end) {
    super(
        // The controller that the command will use
        new PIDController(70, 0, 0),
        // This should return the measurement
        () -> bottomWrist.getEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> {
          // Use the output here
          if(Math.abs(output) > .5) {
            output = Math.signum(output) * .5;
          }
          bottomWrist.setSpeed(output);
        });

        this.end = end;

        addRequirements(bottomWrist);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    super.initialize();
    if(end) {
      this.m_controller.setTolerance(.005);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(end) {
     return  this.m_controller.atSetpoint();
    } else {
      return false;
    }
  }
}
