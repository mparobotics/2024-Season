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
 private final CANSparkMax motor2 = new CANSparkMax(DriveConstants.MOTOR_2_ID, MotorType.kBrushless); 
 //CANSparkMax: the motor, defines the motors. Brushless: the motor we use.

  /** Creates a new MotorSubsystem. */
  
  public RelativeEncoder encoder = motor1.getEncoder(); 
  public RelativeEncoder encoder2 = motor2.getEncoder(); 
  

public Command Run1(DoubleSupplier speed){ //DoubleSupplier: speed here is going to gradually increase/decrease.
  return runOnce(() -> motor1.set(speed.getAsDouble() * Constants.motorSpeedMultiplier)); // return command
}
  public MotorSubsystem(){
      motor1.setInverted(false); 
      motor2.setInverted(true); //Invert the code for motor 1 to have motor 2
      motor2.follow(motor1); //Same code
  }

  public Command RunMotors()
  {
    return runOnce(
      () -> { //have to do
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
}

