// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.topWristConstants;

public class TopWristSubsystem extends SubsystemBase {
  /** Creates a new topWrist. */
  
  private TalonFX topWrist = new TalonFX(topWristConstants.topWristID);  
  DutyCycleEncoder encoder = new DutyCycleEncoder(topWristConstants.topWristAbsoluteEncoder); 
  
  public TopWristSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
