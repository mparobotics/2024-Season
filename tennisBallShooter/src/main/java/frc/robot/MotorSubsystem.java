// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.DriveConstants;

public class MotorSubsystem extends SubsystemBase {
 private final CANSparkMax motor1 = new CANSparkMax(DriveConstants.MOTOR_1_ID, MotorType.kBrushless);
 
  /** Creates a new MotorSubsystem. */
  public RelativeEncoder encoder = motor1.getEncoder();
}

public Command Run1(DoubleSupplier speed){
  return runOnce(() -> motor1.set(speed.getAsDouble() * Constants.motorSpeedMultiplier));
}

  public Command RunMotors()
  {
    return runOnce(
      () -> {
        motor1.set(1);
      }
    );
  }
  public Command StopMotors() 
  {
    return runOnce(
      () -> {
        motor1.set(0);
      
      });
  }
  public Command InverseMotors()
  {
    return runOnce(
      () -> {
        motor1.set(-1);

      }
    ); 
  }

  

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

