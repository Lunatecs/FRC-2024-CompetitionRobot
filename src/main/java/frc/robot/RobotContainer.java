package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    /* Controllers */
    private final Joystick driver = new Joystick(JoystickConstants.DRIVER_USB);
    private final Joystick operator = new Joystick(JoystickConstants.OPERATOR_USB);

    /* Drive Controls */
    private final int translationAxis = JoystickConstants.LEFT_Y_AXIS;
    private final int strafeAxis = JoystickConstants.LEFT_X_AXIS;
    private final int rotationAxis = JoystickConstants.RIGHT_X_AXIS;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, JoystickConstants.BACK_BUTTON);
    private final JoystickButton robotCentric = new JoystickButton(driver, JoystickConstants.LEFT_BUMPER);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final BottomWristSubsystem bottomWristSubsystem = new BottomWristSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        //bottomWristSubsystem.setDefaultCommand(new RunCommand(()->bottomWristSubsystem.setSpeed(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS)),bottomWristSubsystem));
        intakeSubsystem.setDefaultCommand(new RunCommand(() -> intakeSubsystem.setWristSpeed(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS)*.25), intakeSubsystem));
        // Configure the button bindings
        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        new JoystickButton(operator, JoystickConstants.YELLOW_BUTTON).onTrue(new InstantCommand(()->bottomWristSubsystem.setPosition(-26),bottomWristSubsystem));
        new JoystickButton(operator, JoystickConstants.BLUE_BUTTON).onTrue(new InstantCommand(()->bottomWristSubsystem.setPosition(-10),bottomWristSubsystem));
        new JoystickButton(operator, JoystickConstants.GREEN_BUTTON).onTrue(new InstantCommand(()->bottomWristSubsystem.setPosition(-2),bottomWristSubsystem));
        new JoystickButton(driver, JoystickConstants.RIGHT_BUMPER).onTrue(new RunIntakeCommand(intakeSubsystem, shooterSubsystem, 0.1, 0.5))
                                                                .onFalse(new RunIntakeCommand(intakeSubsystem, shooterSubsystem, 0, 0));
    
        new JoystickButton(operator, JoystickConstants.RED_BUTTON).onTrue(new InstantCommand(() -> shooterSubsystem.setshooterSpeed(.75), shooterSubsystem));
        new JoystickButton(operator, JoystickConstants.RIGHT_BUMPER).onTrue(new InstantCommand(() -> {shooterSubsystem.setFeederSpeed(.5); shooterSubsystem.setshooterSpeed(.75);}, shooterSubsystem));
        
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
