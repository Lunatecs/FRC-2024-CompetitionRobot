package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.TargetingUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowMode;
    private BooleanSupplier autoTargeting; 
    private TargetingUtil targetingUtil; 
    private BooleanSupplier middleMode;
    private final double slowSpeed = 0.2;
    private final double slowRotation = 0.2;
    private final double middleSpeed = 0.65;
    private final double middleRotation = 0.65;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowMode, BooleanSupplier autoTargeting, TargetingUtil targetingUtil, BooleanSupplier middleMode) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowMode = slowMode;
        this.autoTargeting = autoTargeting; 
        this.targetingUtil = targetingUtil; 
        this.middleMode = middleMode;
    }

    @Override
    public void execute() {
        String mode = "normal";
        double speedMulti;
        double rotMulti;
        double teleOpMaxSpeed = Constants.Swerve.maxSpeed;
        double teleOpMaxAngularVelocity = Constants.Swerve.maxAngularVelocity;

        speedMulti = teleOpMaxSpeed * .8;
        rotMulti = teleOpMaxAngularVelocity * .8;

        if (slowMode.getAsBoolean()) {
            speedMulti = slowSpeed * teleOpMaxSpeed;
            rotMulti = slowRotation * teleOpMaxAngularVelocity;
            mode="slow";
        }

        if (middleMode.getAsBoolean()) {
            speedMulti = middleSpeed * teleOpMaxSpeed;
            rotMulti = middleRotation * teleOpMaxAngularVelocity;
            mode="middle";
        }


        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if(autoTargeting.getAsBoolean()) {
            rotationVal = targetingUtil.calculateRotation();
            rotMulti = teleOpMaxAngularVelocity;
            mode="auto";
        }


        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(speedMulti), 
            rotationVal * rotMulti, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        SmartDashboard.putString("swerve mode", mode);
    }
}