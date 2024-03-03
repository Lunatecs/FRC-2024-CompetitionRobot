// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.LimelightSubsystem;

/** Add your docs here. */
public class TargetingUtil {

    private LimelightSubsystem limelightSubsystem;
    private PIDController pidController;

    public TargetingUtil(LimelightSubsystem limelightSubsystem){
        this.limelightSubsystem = limelightSubsystem;
        pidController = new PIDController(0.02, 0, 0.0005); 
        pidController.setSetpoint(0);
        pidController.setTolerance(0.25);
    }

    public double calculateRotation(){
      return -MathUtil.clamp(pidController.calculate(limelightSubsystem.GetTx()),-1,1);
    }

    public boolean onTarget(){
        return pidController.atSetpoint();
    }
}
