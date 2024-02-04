// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private TalonFX shooterMotor1;
  private TalonFX shooterMotor2;
  private TalonFX feederMotor;
  private DigitalInput proximitySensor;
  private final VelocityVoltage motorVelocity = new VelocityVoltage(0);

  public ShooterSubsystem() {
    shooterMotor1 = new TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR1);

    shooterMotor2 = new TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR2);
    shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), false)); // Check if shooterMotor2 needs to be inverted

    feederMotor = new TalonFX(Constants.ShooterConstants.FEEDER_MOTOR);

    proximitySensor = new DigitalInput(Constants.ShooterConstants.PROXIMITY_SENSOR);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.0;
    slot0Configs.kP = 1;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    shooterMotor1.getConfigurator().apply(slot0Configs, 0.050);
    motorVelocity.Slot = 0;

    shooterMotor1.setPosition(0);
    shooterMotor2.setPosition(0);
    feederMotor.setPosition(0);

  }

  public void setRPM(int rpm){
    shooterMotor1.setControl(motorVelocity.withVelocity(rpm));
  }

  public void setFeederSpeed(double speed){
    feederMotor.set(speed);
  }

  public boolean getSensor(){
    return proximitySensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
