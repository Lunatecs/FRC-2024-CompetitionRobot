// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TopWristConstants; 

public class TopWristSubsystem extends SubsystemBase {

  private TalonFX topWrist;
  
  //private TalonFXConfiguration topWristConfig = new TalonFXConfiguration();  
  //private DutyCycleEncoder encoder = new DutyCycleEncoder(TopWristConstants.TOP_WRIST_ABSOLUTE_ENCODER); 

  //private final PositionVoltage motorPosition = new PositionVoltage(0);
  final MotionMagicVoltage motionMagicV = new MotionMagicVoltage(0);

  public TopWristSubsystem() {
    topWrist = new TalonFX(TopWristConstants.TOP_WRIST_ID);
    topWrist.getConfigurator().apply(new TalonFXConfiguration()); 

    initializeEncoder();

    var topWristConfig = new TalonFXConfiguration(); 

    var slot0configs = topWristConfig.Slot0;
    slot0configs.kG = 0.25; 
    slot0configs.kS = 0.3;
    slot0configs.kV = 0.1;//.3178;//2.648;//31.78;//.25;
    slot0configs.kA = 0.0; //.02
    slot0configs.kP = 0.00;
    slot0configs.kI = 0.0;
    slot0configs.kD = 0.00;
    slot0configs.GravityType = GravityTypeValue.Arm_Cosine;

    var motionMagicConfig = topWristConfig.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 40;
    motionMagicConfig.MotionMagicAcceleration = 40;

    var feedbackConfig = topWristConfig.Feedback;
    feedbackConfig.SensorToMechanismRatio = 99.9771;

    topWrist.setNeutralMode(NeutralModeValue.Brake);

    //topWrist.getConfigurator().apply(topWristConfig, 0.05);

    motionMagicV.Slot = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Wrist", getEncoder());
  }

  public void setSpeed(double speed){
    topWrist.set(speed);
  }

  public void resetEncoder(){
    topWrist.setPosition(0);
  }

  public void initializeEncoder() {
    //wristMotor1.setPosition(getEncoder());
    topWrist.setPosition(-0.038);
  }

  public double getEncoder() {
    return topWrist.getPosition().getValueAsDouble();
  }

  public void setPosition(double position) {
    topWrist.setControl(motionMagicV.withPosition(position));
  }
}
