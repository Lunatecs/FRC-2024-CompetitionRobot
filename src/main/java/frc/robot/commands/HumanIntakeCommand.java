// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HumanIntakeCommand extends SequentialCommandGroup {
  /** Creates a new HumanIntakeCommand. */
  public HumanIntakeCommand(BottomWristSubsystem bottomWristSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetPivotHighCommand(bottomWristSubsystem, intakeSubsystem, -14),
      new CheckPivotCommand(bottomWristSubsystem, -0.07),
      new ParallelDeadlineGroup(new RunIntakeFromShooterCommand(shooterSubsystem, -.1, -.5), new SetElevatorCommand(elevatorSubsystem, 28, false)),
      new InstantCommand(() -> ledSubsystem.set(ledSubsystem.BLUE)),
      new ParallelDeadlineGroup(new SetPivotBottomCommand(bottomWristSubsystem, intakeSubsystem), new SetElevatorCommand(elevatorSubsystem, 0, true))

    );
  }
}
