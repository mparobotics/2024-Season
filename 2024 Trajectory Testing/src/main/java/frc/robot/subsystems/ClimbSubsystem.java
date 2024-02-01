// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor =  new CANSparkMax(0,MotorType.kBrushless); 
  private final CANSparkMax rightMotor =  new CANSparkMax(0,MotorType.kBrushless); 
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    leftMotor.follow(rightMotor);


  }
  public void setMotorSpeed(double speed){
    rightMotor.set(speed);
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
