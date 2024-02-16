// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPivotHighCommand extends SequentialCommandGroup {
  /** Creates a new SetPivotHighCommand. */
  BottomWristSubsystem bottomWristSubsystem;
  IntakeSubsystem intakeSubsystem;
  public SetPivotHighCommand(BottomWristSubsystem bottomWristSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetIntakeWristPosition(.32, 0, 0, 0.5 , -4, true, intakeSubsystem),
      new InstantCommand(()->bottomWristSubsystem.setPosition(-26),bottomWristSubsystem)
    );

    addRequirements(bottomWristSubsystem, intakeSubsystem);
  }
}