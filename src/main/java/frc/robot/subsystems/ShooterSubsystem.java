// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  public TalonFXConfiguration shooterMotor1Config = new TalonFXConfiguration();
  public TalonFXConfiguration shooterMotor2Config = new TalonFXConfiguration();
  public TalonFXConfiguration feederMotorConfiguration = new TalonFXConfiguration();
  /** Creates a new ShooterSubsystem. */
  private TalonFX shooterMotor1;

  private TalonFX shooterMotor2;

  private TalonFX feederMotor;
  private TalonFXConfiguration feederMotorConfig;

  public ShooterSubsystem() {
    shooterMotor1=new TalonFX(0);

    shooterMotor2=new TalonFX(0);

    feederMotor=new TalonFX(0);

    shooterMotor1.getConfigurator().apply(shooterMotor1Config);
    shooterMotor2.getConfigurator().apply(shooterMotor2Config);
    feederMotor.getConfigurator().apply(feederMotorConfig);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
