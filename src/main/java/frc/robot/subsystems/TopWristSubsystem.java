// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TOP_WRIST_CONSTANTS; 

public class TopWristSubsystem extends SubsystemBase {
  /** Creates a new topWrist. */
  
  public TalonFXConfiguration topWristConfig = new TalonFXConfiguration(); 
  private TalonFX topWrist = new TalonFX(TOP_WRIST_CONSTANTS.TOP_WRIST_ID);  
  DutyCycleEncoder encoder = new DutyCycleEncoder(TOP_WRIST_CONSTANTS.TOP_WRIST_ABSOLUTE_ENCODER); 

  public TopWristSubsystem() {
    topWrist.getConfigurator().apply(topWristConfig); 
    topWrist.setNeutralMode(NeutralModeValue.Brake);

      encoder.reset();  


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){

    topWrist.set(speed);

  }

  public void resetEncoder(){

    encoder.reset();

  }


}
