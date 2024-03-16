// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BottomWristConstants;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TopWristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpRetractCommand extends SequentialCommandGroup {

  private IntakeSubsystem intakeSubsystem;
  private TopWristSubsystem topWristSubsystem;
  private BottomWristSubsystem bottomWristSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  /** Creates a new AmpRetractCommand. */
  public AmpRetractCommand(IntakeSubsystem intakeSubsystem, TopWristSubsystem topWristSubsystem, BottomWristSubsystem bottomWristSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ParallelRaceGroup(new SetTopWristCommand(topWristSubsystem, -.02,true),new SetElevatorCommand(elevatorSubsystem, 0, false),new SetBottomWristCommand(bottomWristSubsystem, -.076, false)),//-.11 old position for old bottom wrist gearing
      new InstantCommand(() -> topWristSubsystem.setSpeed(0), topWristSubsystem),
      new ParallelRaceGroup(new SetElevatorCommand(elevatorSubsystem, 0, true),new SetBottomWristCommand(bottomWristSubsystem, -.076, false)),//-.11 old position for old bottom wrist gearing
      new InstantCommand(() -> elevatorSubsystem.setSpeed(0), elevatorSubsystem),
      new SetBottomWristCommand(bottomWristSubsystem, 0,true),
      new InstantCommand(() -> bottomWristSubsystem.setSpeed(0), bottomWristSubsystem),
      new RetractIntakeCommand(intakeSubsystem)
    );
  }
}
