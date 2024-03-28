// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
    private TalonFX feederMotor;
    private DigitalInput proximitySensor;

  public FeederSubsystem() {

    feederMotor = new TalonFX(Constants.ShooterConstants.FEEDER_MOTOR);
    proximitySensor = new DigitalInput(Constants.ShooterConstants.PROXIMITY_SENSOR);
    feederMotor.getConfigurator().apply(new TalonFXConfiguration());

    feederMotor.setPosition(0);

    feederMotor.setInverted(true);
  }

  public void setSpeed(double speed){
    feederMotor.set(speed);
  }

  public boolean getSensor(){
    return proximitySensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("prox sensor", getSensor());
  }
}
