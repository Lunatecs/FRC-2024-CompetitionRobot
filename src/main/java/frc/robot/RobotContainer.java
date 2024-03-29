package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.TargetingUtil;

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

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final BottomWristSubsystem bottomWristSubsystem = new BottomWristSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final TopWristSubsystem topWristSubsystem = new TopWristSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem(); 
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem(); 

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false, //driver.getRawButton(Constants.JoystickConstants.RIGHT_BUMPER), //false for field centric
                () -> driver.getRawButton(JoystickConstants.LEFT_BUMPER), // slowMode
                () -> driver.getRawButton(JoystickConstants.GREEN_BUTTON),  // auto targeting
                new TargetingUtil(limelightSubsystem),
                () -> driver.getRawButton(JoystickConstants.RIGHT_BUMPER) // middleMode
            )
        );

        // Configure the button bindings
        configureButtonBindings();
        
        // Auto Configurations
        NamedCommands.registerCommand("shootNote", new ShootNoteCommand(shooterSubsystem, ledSubsystem, 60, feederSubsystem));
        NamedCommands.registerCommand("shootNoteFar", new ShootNoteCommand(shooterSubsystem, ledSubsystem, 80, feederSubsystem));
        NamedCommands.registerCommand("runShooter", new InstantCommand(() -> shooterSubsystem.setRPM(80), shooterSubsystem));
        NamedCommands.registerCommand("shootNoteLine", new AutoShootNoteLineCommand(shooterSubsystem, 70, feederSubsystem));
        NamedCommands.registerCommand("raisePivot26", new InstantCommand(()->bottomWristSubsystem.setPosition(-18.056),bottomWristSubsystem));
        NamedCommands.registerCommand("raisePivotNoteLine", new InstantCommand(()->bottomWristSubsystem.setPosition(-9.722),bottomWristSubsystem));
        NamedCommands.registerCommand("raisePivot8", new InstantCommand(()->bottomWristSubsystem.setPosition(-4.5),bottomWristSubsystem)); //-5.5 //-5.8 //1.5
        NamedCommands.registerCommand("checkPivot8", new CheckPivotCommand(bottomWristSubsystem, .01));
        NamedCommands.registerCommand("raisePivotHigh", new InstantCommand(() -> bottomWristSubsystem.setPosition(-20), bottomWristSubsystem));
        NamedCommands.registerCommand("checkPivotHigh", new CheckPivotCommand(bottomWristSubsystem, .105));

        
        NamedCommands.registerCommand("checkPivot26", new CheckPivotCommand(bottomWristSubsystem, .09));
        NamedCommands.registerCommand("checkPivotNoteLine", new CheckPivotCommand(bottomWristSubsystem, .04)); //needs to be checked
        NamedCommands.registerCommand("PivotBrake", new InstantCommand(() -> bottomWristSubsystem.setNeutralMode(NeutralModeValue.Brake)));

        NamedCommands.registerCommand("RunIntake", new RunIntakeCommand(intakeSubsystem, feederSubsystem, 0.2, 1, false));
        NamedCommands.registerCommand("DropIntake", new SetIntakeWristPosition(.32, 0, 0, 0.5 , -9.04, true, intakeSubsystem)); //-10.6
        NamedCommands.registerCommand("SetPivot0", new InstantCommand(()->bottomWristSubsystem.setPosition(-2),bottomWristSubsystem));
        NamedCommands.registerCommand("ZeroPower", new InstantCommand(() -> bottomWristSubsystem.setSpeed(0.0), bottomWristSubsystem));

        NamedCommands.registerCommand("AutoTargeting", new AutoRotateToTargetCommand(s_Swerve, limelightSubsystem));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.addOption("Do Nothing", new AutoDoNothing());
        autoChooser.addOption("Four Piece in Front", new PathPlannerAuto("Four Piece Base Auto"));
        autoChooser.addOption("Move 1 Meter", new PathPlannerAuto("1 Meter Forward"));

    }

    public void teleopInit() {
        bottomWristSubsystem.setSpeed(0.0);
        feederSubsystem.setSpeed(0.0);
        shooterSubsystem.setshooterSpeed(0.0);
        intakeSubsystem.setIntakeSpeed(0.0);
        bottomWristSubsystem.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Controls */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

       // new POVButton(driver, JoystickConstants.POV_UP).onTrue(new RunIntakeFromShooterCommand(shooterSubsystem, -0.1, -.5))
                       //                               .onFalse(new SequentialCommandGroup(new InstantCommand(() -> shooterSubsystem.setshooterSpeed(0)), new InstantCommand(() -> shooterSubsystem.setFeederSpeed(0))));
        
        new POVButton(driver, JoystickConstants.POV_UP).onTrue(new HumanIntakeCommand(bottomWristSubsystem, intakeSubsystem, shooterSubsystem, elevatorSubsystem, ledSubsystem, feederSubsystem));

        new POVButton(operator, JoystickConstants.POV_LEFT).onTrue(new InstantCommand(() -> climberSubsystem.setSpeed(0.5)))
                                                            .onFalse(new InstantCommand(() -> climberSubsystem.setSpeed(0)));

        new POVButton(operator, JoystickConstants.POV_RIGHT).onTrue(new InstantCommand(() -> climberSubsystem.setSpeed(-0.5)))
                                                            .onFalse(new InstantCommand(() -> climberSubsystem.setSpeed(0)));
        //new JoystickButton(operator, JoystickConstants.BACK_BUTTON).onTrue(new InstantCommand(()->ledSubsystem.set(0, 0, 255))).onFalse(new InstantCommand(()->ledSubsystem.set(255, 0, 0)));

        //Intake                                                                          
        new Trigger(() -> {return feederSubsystem.getSensor() && Math.abs(driver.getRawAxis(JoystickConstants.RIGHT_TRIGGER)) > 0.1;}).onTrue(new ExtendAndRunIntakeCommand(intakeSubsystem, feederSubsystem, ledSubsystem));
                                                                                                       // .onFalse(new RetractIntakeCommand(intakeSubsystem));
        new JoystickButton(driver, JoystickConstants.RED_BUTTON).onTrue(new RetractIntakeCommand(intakeSubsystem));
        new Trigger(() -> {return Math.abs(driver.getRawAxis(JoystickConstants.LEFT_TRIGGER)) > 0.1;}).onTrue(new RunIntakeCommand(intakeSubsystem, feederSubsystem, 0.2, 1, true))  
                                                                                                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0), intakeSubsystem), new RetractIntakeCommand(intakeSubsystem)));
                                
        /*Outtake                                                                    
        new Trigger(() -> {return Math.abs(driver.getRawAxis(JoystickConstants.RIGHT_TRIGGER)) > 0.1;}).onTrue(new ExtendAndRunIntakeCommand(intakeSubsystem, shooterSubsystem))
                                                                                                       .onFalse(new RetractIntakeCommand(intakeSubsystem, shooterSubsystem)); */
    
        //new JoystickButton(driver, JoystickConstants.GREEN_BUTTON).onTrue(new SetIntakeWristPosition(.32, 0, 0, 0.5, -3.0, intakeSubsystem)); // Down Position
        //new JoystickButton(driver, JoystickConstants.BLUE_BUTTON).onTrue(new RetractIntakeCommand(intakeSubsystem));
        
        new JoystickButton(driver, JoystickConstants.YELLOW_BUTTON).onTrue(new InstantCommand(() -> feederSubsystem.setSpeed(-.25), feederSubsystem))
                                                                    .onFalse(new InstantCommand(() -> feederSubsystem.setSpeed(0.0), feederSubsystem));

        new JoystickButton(driver, JoystickConstants.GREEN_BUTTON).onTrue(new SequentialCommandGroup(new SetIntakeWristPosition(.32, 0, 0, 0.5 , -4, true, intakeSubsystem), new AutoRangePIDCommand(bottomWristSubsystem, limelightSubsystem, ledSubsystem, feederSubsystem)))
                                                                    .onFalse(new SequentialCommandGroup( new InstantCommand(() -> { if(!feederSubsystem.getSensor()) {
                                                                                                                                        ledSubsystem.set(ledSubsystem.BLUE);
                                                                                                                                    } else {
                                                                                                                                        ledSubsystem.set(ledSubsystem.YELLOW);
                                                                                                                                    } 
                                                                                                                                } ), new SetPivotBottomCommand(bottomWristSubsystem, intakeSubsystem)));

        /* Operator Controls */
        new JoystickButton(operator, JoystickConstants.YELLOW_BUTTON).onTrue(new SetPivotHighCommand(bottomWristSubsystem, intakeSubsystem, -3.646));//-5.25 old position for old bottom wrist gearing
        //new JoystickButton(operator, JoystickConstants.BLUE_BUTTON).onTrue(new SetPivotHighCommand(bottomWristSubsystem, intakeSubsystem, -9.722));//-14 old position for old bottom wrist gearing
        //new JoystickButton(operator, JoystickConstants.BLUE_BUTTON).onTrue(new InstantCommand(()->bottomWristSubsystem.setPosition(-10),bottomWristSubsystem));
        new JoystickButton(operator, JoystickConstants.GREEN_BUTTON).onTrue(new SetPivotBottomCommand(bottomWristSubsystem, intakeSubsystem));
        new JoystickButton(operator, JoystickConstants.RED_BUTTON).onTrue(new SetPivotHighCommand(bottomWristSubsystem, intakeSubsystem, -19.5));//-26 old position for old bottom wrist gearing //18.056

        new JoystickButton(operator, JoystickConstants.BLUE_BUTTON).onTrue(new SetTopWristCommand(topWristSubsystem, -.09, false))
                                                                .onFalse(new SequentialCommandGroup(new SetTopWristCommand(topWristSubsystem, -.02,true), new InstantCommand(() -> topWristSubsystem.setSpeed(0), topWristSubsystem)));
                                                                    

        //new POVButton(operator, JoystickConstants.POV_DOWN).onTrue(new SetPivotBaseCommand(bottomWristSubsystem))
                                         //                   .onFalse(new InstantCommand(() -> bottomWristSubsystem.setSpeed(0),bottomWristSubsystem));

        //new POVButton(operator, JoystickConstants.POV_DOWN).onTrue(new SequentialCommandGroup(new SetIntakeWristPosition(.32, 0, 0, 0.5 , -4, true, intakeSubsystem), new AutoRangePIDCommand(bottomWristSubsystem, limelightSubsystem)));
       // new POVButton(operator, JoystickConstants.POV_DOWN).onTrue(new SetIntakeWristPosition(.32, 0, 0, 0.5 , -4, true, intakeSubsystem)).whileTrue(new AutoRangeCommand(limelightSubsystem, bottomWristSubsystem));
    
        new POVButton(operator, JoystickConstants.POV_UP).onTrue(new AmpShootCommand(elevatorSubsystem, intakeSubsystem, topWristSubsystem, bottomWristSubsystem))
                                                        .onFalse(new AmpRetractCommand(intakeSubsystem, topWristSubsystem, bottomWristSubsystem, elevatorSubsystem));                     
                                                        //.onFalse(new SequentialCommandGroup(new InstantCommand(() -> topWristSubsystem.setPosition(0)),new WaitCommand(1),new InstantCommand(() -> topWristSubsystem.setSpeed(0))));
        
        //new POVButton(operator, JoystickConstants.POV_LEFT).onTrue(new SetClimbPositionCommand(intakeSubsystem, bottomWristSubsystem, elevatorSubsystem));
        //new POVButton(operator, JoystickConstants.POV_RIGHT).onTrue(new ClimbCommand(intakeSubsystem, bottomWristSubsystem, elevatorSubsystem));

        new Trigger(() -> {return Math.abs(operator.getRawAxis(JoystickConstants.LEFT_TRIGGER))> 0.1;}).onTrue(new ShootNoteCommand(shooterSubsystem, ledSubsystem, 20, feederSubsystem));
        new JoystickButton(operator, JoystickConstants.RIGHT_BUMPER).onTrue(new ShootNoteCommand(shooterSubsystem, ledSubsystem, 60, feederSubsystem));
        new JoystickButton(operator, JoystickConstants.LEFT_BUMPER).onTrue(new ShootNoteCommand(shooterSubsystem, ledSubsystem, 90, feederSubsystem));
       // new JoystickButton(operator, JoystickConstants.START_BUTTON).onTrue(new SetIntakeWristPosition(.32, 0, 0, 0.5 , -0.3)); //Up Position
       // new JoystickButton(operator, JoystickConstants.START_BUTTON).onTrue(new SequentialCommandGroup(new SetIntakeWristPosition(.32, 0, 0, 0.5 , -4, true, intakeSubsystem), new AutoRangeCommand(limelightSubsystem, bottomWristSubsystem)));
       new JoystickButton(operator, JoystickConstants.START_BUTTON).onTrue(new SetPivotBaseCommand(bottomWristSubsystem))
                                                                    .onFalse(new InstantCommand(() -> bottomWristSubsystem.setSpeed(0),bottomWristSubsystem));
       //Elevator
       new Trigger(() -> {return Math.abs(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS))> 0.1;}).onTrue(new InstantCommand(() -> elevatorSubsystem.setSpeed(0.3*Math.signum(operator.getRawAxis(JoystickConstants.LEFT_Y_AXIS))), elevatorSubsystem))
                                                                                                    .onFalse(new InstantCommand(()->elevatorSubsystem.setSpeed(0),elevatorSubsystem));

       //Top Wrist
      // new Trigger(() -> {return Math.abs(operator.getRawAxis(JoystickConstants.RIGHT_Y_AXIS))> 0.1;}).onTrue(new SequentialCommandGroup(new SetIntakeWristPosition(.32, 0, 0, 0.5 , -4, true, intakeSubsystem), new InstantCommand(() -> topWristSubsystem.setSpeed(0.5*Math.signum(operator.getRawAxis(JoystickConstants.RIGHT_Y_AXIS))), topWristSubsystem)))
      //                                                                                              .onFalse(new ParallelCommandGroup(new RetractIntakeCommand(intakeSubsystem), new InstantCommand(()->topWristSubsystem.setSpeed(0),topWristSubsystem)));

      new Trigger(() -> {return Math.abs(operator.getRawAxis(JoystickConstants.RIGHT_Y_AXIS))> 0.1;}).onTrue(new InstantCommand(() -> topWristSubsystem.setSpeed(0.5*Math.signum(operator.getRawAxis(JoystickConstants.RIGHT_Y_AXIS))), topWristSubsystem))
                                                                                                    .onFalse(new InstantCommand(()->topWristSubsystem.setSpeed(0),topWristSubsystem));   

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
