// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbCommand extends SequentialCommandGroup {
  /** Creates a new ClimbCommand. */
  public ClimbCommand(IntakeSubsystem intakeSubsystem, BottomWristSubsystem bottomWristSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorCommand(elevatorSubsystem, 1, true),
      new InstantCommand(() -> elevatorSubsystem.setSpeed(0), elevatorSubsystem),
      new SetBottomWristCommand(bottomWristSubsystem, 0,true),
      new InstantCommand(() -> bottomWristSubsystem.setNeutralMode(NeutralModeValue.Brake), bottomWristSubsystem)
    );
  }
}
