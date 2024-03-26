// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
  //private CANdle candle = new CANdle(31);
  //private CANdleConfiguration config = new CANdleConfiguration();
  /** Creates a new LEDSubsystem. */
  
  private int numberOfLEDs=23; //16

  public final Color GOLDEN = new Color(170, 170, 0,0); 
  public final Color YELLOW = new Color(255,255,0,0);
  public final Color BLUE = new Color(0,0,255,4);
  public final Color GREEN = new Color(0,170,0,4);

  private Color setColor;

  public LEDSubsystem() {
   
    
    //config.v5Enabled=true;
    //config.stripType = LEDStripType.RGB; // set the strip type to RGB
    //config.brightnessScalar = 1.0; // dim the LEDs to half brightness
    //candle.configAllSettings(config);
    set(YELLOW);
    
  }

  private void set(int R, int G, int B){
    //candle.setLEDs(R,G,B);
  
    //candle.setLEDs(R, G, B, 0, 0, 14);
    //candle.setLEDs(R, G, B);
    //RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 16);

    //candle.animate(rainbowAnim);
    
  } 

  public void set(Color color) {
    /*
    setColor=color;
  //set(color.red,color.green,color.blue);
    if(color.equals(this.YELLOW)) {
      candle.animate(new TwinkleAnimation(color.red, color.green, color.blue, 0, .5, numberOfLEDs, TwinklePercent.Percent30));
    } else if(color.equals(this.BLUE)) {
      candle.animate(new StrobeAnimation(color.red, color.green, color.blue, 0, .5, numberOfLEDs));
    } else {
      set(color.red,color.green,color.blue);
    }
    */
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //ErrorCode error = candle.getLastError(); // gets the last error generated by the CANdle

    //CANdleFaults faults = new CANdleFaults();

    //ErrorCode faultsError = candle.getFaults(faults);

    //SmartDashboard.putString("Errors", error.toString());
    //SmartDashboard.putString("Faults",faultsError.toString());
  }

  public class Color {

    public final int red;
    public final int green;
    public final int blue;
    public final int priority;

    public Color(int red, int green, int blue, int priority) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.priority = priority;
    }

    public boolean equals(Color c) {
      if(this.red==c.red && this.green == c.green && this.blue == c.blue && this.priority == c.priority) {
        return  true;
      } else {
        return false;
      }
    }

  }

}
