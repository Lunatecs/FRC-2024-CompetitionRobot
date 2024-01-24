// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsytem extends SubsystemBase {
  
  TalonFX elevatorMotor;
  public ElevatorSubsytem() {
    elevatorMotor = new TalonFX(2);
  }

  public void runElevator(double speed) {
    elevatorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
