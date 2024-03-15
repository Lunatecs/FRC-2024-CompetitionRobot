// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private TalonFX climberMotor; 

  public ClimberSubsystem() {
    climberMotor = new TalonFX(0); //add ID later
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
    initializeEncoder();

  }

  public void initializeEncoder(){
    climberMotor.setPosition(0);
  }

  public void setSpeed(double speed){
    climberMotor.set(speed); 
  }

  public double getEncoder(){
    return climberMotor.getPosition().getValueAsDouble(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
