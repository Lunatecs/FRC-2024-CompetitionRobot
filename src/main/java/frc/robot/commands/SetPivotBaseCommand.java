// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.BottomWristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPivotBaseCommand extends PIDCommand {
  /** Creates a new SetPivotBaseCommand. */
  public SetPivotBaseCommand(BottomWristSubsystem bottomWrist) {
    super(
        // The controller that the command will use
        new PIDController(1.8, 0, 0),
        // This should return the measurement
        () -> bottomWrist.getEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> 0.047222,
        // This uses the output
        output -> {
          if(Math.abs(output) > 0.5) {
            output = 0.5*Math.signum(output);
          }
          bottomWrist.setSpeed(output);
          SmartDashboard.putNumber("down", output);
        });
        addRequirements(bottomWrist);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
