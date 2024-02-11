// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFXConfiguration intakeMotorConfig= new TalonFXConfiguration();
  private TalonFXConfiguration wristMotorConfig= new TalonFXConfiguration();

  private TalonFX intakeMotor=new TalonFX(IntakeConstants.INTAKE_MOTOR);
  private TalonFX wristMotor=new TalonFX(IntakeConstants.WRIST_MOTOR);
  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0); // TO DO; fix value

  public IntakeSubsystem() {
    intakeMotor.getConfigurator().apply(intakeMotorConfig); 
    wristMotor.getConfigurator().apply(wristMotorConfig);

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    
    wristMotor.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Position", wristMotor.getPosition().getValueAsDouble());
 
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void setWristSpeed(double speed){
    wristMotor.set(speed);
  }

  public double getWristEncoder(){
    return wristMotor.getPosition().refresh().getValue();
  }

  public double getAbsoluteEncoder(){
    return absoluteEncoder.getAbsolutePosition();

  }
}