// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
  private CANdle candle;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    candle = new CANdle(31);
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 1.0; // dim the LEDs to half brightness
    candle.configAllSettings(config);
    
  }

  public void set(int R, int G, int B){
    candle.setLEDs(R,G,B);
    candle.setLEDs(R, G, B, 0, 0, 8);
  } 

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
