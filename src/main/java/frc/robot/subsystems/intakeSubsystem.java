// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem extends SubsystemBase {

  public TalonFXConfiguration intakeMotorConfig= new TalonFXConfiguration();
  public TalonFXConfiguration wristMotorConfig= new TalonFXConfiguration();
  /** Creates a new intakeSubsystem. */
  private TalonFX intakeMotor=new TalonFX(0); //TO DO; fix id value
  private TalonFX wristMotor=new TalonFX(0); // TO DO; fix id value
  private DutyCycleEncoder encoder = new DutyCycleEncoder(0); // TO DO; fix value

  public intakeSubsystem() {
    intakeMotor.getConfigurator().apply(intakeMotorConfig); 
    wristMotor.getConfigurator().apply(wristMotorConfig);

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 
  }

  public void setIntakeSpeed(double speed){

    intakeMotor.set(speed);
  }

  public void setWristSpeed(double speed){

    wristMotor.set(speed);
  }

  public double getEncoder(){
    return wristMotor.getPosition().refresh().getValue();
  }

  public double getAbsoluteEncoder(){
    return encoder.getAbsolutePosition();
  }
}
