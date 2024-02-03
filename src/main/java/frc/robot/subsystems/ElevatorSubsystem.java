// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  /** Creates a new ElevatorSubsystem. */
  private TalonFX elevatorMotor;
  private DutyCycleEncoder absoluteEncoder;
  private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  private PositionVoltage motorPosition = new PositionVoltage(0);

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID);
    
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    elevatorMotor.getConfigurator().apply(elevatorConfig, 0.050); // sets elevatorMotor to factory default config settings
    
    motorPosition.Slot = 0;
    absoluteEncoder = new DutyCycleEncoder(new DigitalInput(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID));
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake); 
    initializeEncoder();
  }

  public void setPosition(double rotations) {
    elevatorMotor.setControl(motorPosition.withPosition(rotations));
  }

  public void initializeEncoder() {
    elevatorMotor.setPosition(absoluteEncoder.getAbsolutePosition());
  }

  @Override
  public void periodic() {}
  
}
