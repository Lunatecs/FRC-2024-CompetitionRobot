// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BottomWristSubsystem extends SubsystemBase {

  private TalonFX wristMotor1;
  private TalonFX wristMotor2;
  private DutyCycleEncoder absoluteEncoder;
  private double max = -1;
  
  private final PositionVoltage motorPosition = new PositionVoltage(0);
  final MotionMagicVoltage motionMagicV = new MotionMagicVoltage(0);

  public BottomWristSubsystem() {

    wristMotor1 = new TalonFX(Constants.BottomWristConstants.WRIST_MOTOR1);
    wristMotor1.getConfigurator().apply(new TalonFXConfiguration());
    
    wristMotor2 = new TalonFX(Constants.BottomWristConstants.WRIST_MOTOR2);
    wristMotor2.getConfigurator().apply(new TalonFXConfiguration());

    //absoluteEncoder = new DutyCycleEncoder(new DigitalInput(Constants.BottomWristConstants.ABSOLUTE_ENCODER));
    
    wristMotor2.setControl(new Follower(wristMotor1.getDeviceID(), true));

    
    initializeEncoder();

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0configs = talonFXConfigs.Slot0;
    slot0configs.kG = -.19; //-.15;
    slot0configs.kS = -.223; //-.35;//-.41;//-.3;
    slot0configs.kV =  .12;//.118//.12//.11//.1265//0.15  -old gear ratio  ;//.3178;//2.648;//31.78;//.25;
    slot0configs.kA = 0.0; //.02
    slot0configs.kP = 0.00;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.00;
    slot0configs.GravityType = GravityTypeValue.Arm_Cosine;


    var slot1configs = talonFXConfigs.Slot1;
    slot1configs.kG = -.19; //-.15;
    slot1configs.kS = -.35; //-.41;//-.3;
    slot1configs.kV =  .1265;//0.15  -old gear ratio  ;//.3178;//2.648;//31.78;//.25;
    slot1configs.kA = 0.0; //.02
    slot1configs.kP = 0.00;
    slot1configs.kI = 0.0;
    slot1configs.kD = 0.00;
    slot1configs.GravityType = GravityTypeValue.Arm_Cosine;


    var motionMagicConfig = talonFXConfigs.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 68;
    motionMagicConfig.MotionMagicAcceleration = 68;

    var feedbackConfig = talonFXConfigs.Feedback;
    feedbackConfig.SensorToMechanismRatio = 195.556;//281.6 old gearing ratio for bottom wrist
    //feedbackConfig.RotorToSensorRatio = 195.555;
    
    wristMotor1.setNeutralMode(NeutralModeValue.Brake);
    wristMotor2.setNeutralMode(NeutralModeValue.Brake);


    wristMotor1.getConfigurator().apply(talonFXConfigs, 0.05);



    motionMagicV.Slot = 0;
  }

  public double getEncoder() {
    return wristMotor1.getPosition().getValueAsDouble();
  }

  public void initializeEncoder() {
    //wristMotor1.setPosition(getEncoder());
    wristMotor1.setPosition(0.047222);
  }

  public void initializeEncoderToZero() {
    //wristMotor1.setPosition(getEncoder());
    wristMotor1.setPosition(0.0,.1);
  }

// Pass parameters through setPosition
  public void setPosition(double position) {
    wristMotor1.setControl(motionMagicV.withPosition(position));
  }

  public void setSpeed(double speed){
    wristMotor1.set(speed);
  }

  public void setNeutralMode(NeutralModeValue value) {
    wristMotor1.setNeutralMode(value);
    wristMotor2.setNeutralMode(value);
  }

  public double getCurrent() {
    return wristMotor1.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    double vel = wristMotor1.getVelocity().getValue();
    if(Math.abs(vel) > max) {
      max=Math.abs(vel);
    }
    SmartDashboard.putNumber("BottomWrist", getEncoder());
    SmartDashboard.putNumber("BottomWrist Rotations", getEncoder()*195.556);
    SmartDashboard.putNumber("Bottom Current", getCurrent());
   // System.out.println("Position: " + this.wristMotor1.getPosition().getValueAsDouble() * 281.6 + "Angle:" + this.wristMotor1.getPosition().getValueAsDouble()*360 + "Vel: " + vel + " Max: " + max + " Voltage: " + this.wristMotor1.getMotorVoltage());
  }
  
}
