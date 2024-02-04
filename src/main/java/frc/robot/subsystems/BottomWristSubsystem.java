// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BottomWristSubsystem extends SubsystemBase {

  private TalonFX wristMotor1;
  private TalonFX wristMotor2;
  private DutyCycleEncoder absoluteEncoder;
  
  private final PositionVoltage motorPosition = new PositionVoltage(0);

  public BottomWristSubsystem() {

    wristMotor1 = new TalonFX(Constants.BottomWristConstants.WRIST_MOTOR1);
    wristMotor1.getConfigurator().apply(new TalonFXConfiguration());
    wristMotor2 = new TalonFX(Constants.BottomWristConstants.WRIST_MOTOR2);
    wristMotor2.getConfigurator().apply(new TalonFXConfiguration());
    absoluteEncoder = new DutyCycleEncoder(new DigitalInput(Constants.BottomWristConstants.ABSOLUTE_ENCODER));
    wristMotor2.setControl(new Follower(wristMotor1.getDeviceID(), true));
    
    initializeEncoder();

    var slot0configs = new Slot0Configs();
    slot0configs.kP = 1.0;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.00;
    
    wristMotor1.getConfigurator().apply(slot0configs, 0.05);

    motorPosition.Slot = 0;
  }

  public double getEncoder() {
 
    return absoluteEncoder.getAbsolutePosition();
  }

  public void initializeEncoder() {

    wristMotor1.setPosition(getEncoder());
  }
// Pass parameters through setPosition
  public void setPosition() {
    
    wristMotor1.setControl(motorPosition.withPosition(50));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
