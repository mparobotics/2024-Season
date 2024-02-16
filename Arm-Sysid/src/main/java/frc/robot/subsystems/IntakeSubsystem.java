// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkFlex intakeMotor = new CANSparkFlex(20, MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */  

  public IntakeSubsystem() {

  }
  public Command controlIntakeWithJoystick(DoubleSupplier speed){
    return runOnce(() -> {
      if(Math.abs(speed.getAsDouble()) < 0.1){
        intakeMotor.set(0);
      }
      else{
        intakeMotor.set(speed.getAsDouble());
      }
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
