// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomWristSubsystem extends SubsystemBase {

  private TalonFX wristMotor1;

  private TalonFX wristMotor2;

  private DutyCycleEncoder absoluteEncoder;
  
  final PositionVoltage motorPosition = new PositionVoltage(0);

  public BottomWristSubsystem() {

    wristMotor1 = new TalonFX(0);

    wristMotor2 = new TalonFX(1);
    
    absoluteEncoder = new DutyCycleEncoder(new DigitalInput(0));
    
    wristMotor2.setControl(new Follower(wristMotor1.getDeviceID(), true));
    
    initializeEncoder();

    var slot0configs = new Slot0Configs();
    
    slot0configs.kP = 0.11;
    
    slot0configs.kI = 0.48;
    
    slot0configs.kD = 0.01;
    
    wristMotor1.getConfigurator().apply(slot0configs, 0.05);

    motorPosition.Slot = 0;
  }

  public double getEncoder() {
 
    return absoluteEncoder.getAbsolutePosition();
  }

  public void initializeEncoder() {

    wristMotor1.setPosition(getEncoder());
  }

  public void setPosition() {
    
    wristMotor1.setControl(motorPosition.withPosition(50));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
