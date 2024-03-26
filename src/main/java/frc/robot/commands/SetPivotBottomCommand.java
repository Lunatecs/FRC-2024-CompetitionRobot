// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPivotBottomCommand extends SequentialCommandGroup {
  /** Creates a new SetPivotBottomCommand. */
  BottomWristSubsystem bottomWristSubsystem;
  IntakeSubsystem intakeSubsystem;
  public SetPivotBottomCommand(BottomWristSubsystem bottomWristSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->bottomWristSubsystem.setPosition(-1.389),bottomWristSubsystem),//-2 old position for old bottom wrist gearing
      new WaitCommand(.25),
      //new CheckPivotCommand(bottomWristSubsystem, -.01),
      new ResetBottomWrist(bottomWristSubsystem),
      new InstantCommand(() ->bottomWristSubsystem.setSpeed(0), bottomWristSubsystem),
      new InstantCommand(() -> bottomWristSubsystem.initializeEncoderToZero(), bottomWristSubsystem),
      new RetractIntakeCommand(intakeSubsystem)
    );

    addRequirements(intakeSubsystem, bottomWristSubsystem);
  }
}
