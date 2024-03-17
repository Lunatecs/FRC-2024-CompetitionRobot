// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteCommand extends SequentialCommandGroup {
  /** Creates a new ShootNoteCommand. */
  public ShootNoteCommand(ShooterSubsystem shooter, LEDSubsystem led, int velocity) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunShooterCommand(shooter, velocity),
      new InstantCommand(() -> shooter.setFeederSpeed(0.75), shooter),
      new WaitCommand(0.5),
      new InstantCommand(() -> shooter.setFeederSpeed(0.0), shooter),
      new InstantCommand(() -> shooter.setshooterSpeed(0.0), shooter),
      new InstantCommand(() -> led.set(led.GOLDEN))
    );
    addRequirements(shooter);
  }
}
