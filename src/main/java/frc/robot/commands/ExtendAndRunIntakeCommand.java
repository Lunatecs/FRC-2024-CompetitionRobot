// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendAndRunIntakeCommand extends SequentialCommandGroup {
  /** Creates a new ExtendAndRunIntakeCommand. */
  public ExtendAndRunIntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, LEDSubsystem led) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    addCommands(
      new SequentialCommandGroup(new ParallelRaceGroup(new SetIntakeWristPosition(.32, 0, 0, 0.5 , -8.0, true, intake), new WaitCommand(0.7)), new InstantCommand(() -> intake.setWristSpeed(0))),
      //new SetIntakeWristPosition(.32, 0, 0, 0.5 , -9.04, true, intake), //10.6
      new RunIntakeCommand(intake, feeder, 0.2, 1, false),
      new InstantCommand(() -> led.set(led.BLUE)),
      new RetractIntakeCommand(intake)
      );
    addRequirements(intake, feeder);
  }

  /*
  public ExtendAndRunIntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter, double intakeSpeed) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new InstantCommand(() -> intake.setIntakeSpeed(-1)));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetIntakeWristPosition(.32, 0, 0, 0.5 , -0.3, intake, shooter));
      
  }
*/
}
