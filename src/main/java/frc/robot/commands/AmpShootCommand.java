// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BottomWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TopWristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShootCommand extends ParallelCommandGroup {

  private ElevatorSubsystem elevatorSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TopWristSubsystem topWristSubsystem;
  private BottomWristSubsystem bottomWristSubsystem;

  /** Creates a new AmpShootCommand. */
  public AmpShootCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, TopWristSubsystem topWristSubsystem, BottomWristSubsystem bottomWristSubsystem) {
    


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetIntakeWristPosition(.32, 0, 0, 0.5 , -4, true, intakeSubsystem), 
      new SetTopWristCommand(topWristSubsystem, -.275,false), //-.369
      new SetBottomWristCommand(bottomWristSubsystem, -.11,false),
      new SetElevatorCommand(elevatorSubsystem, 30, false)
    );
  }
}
