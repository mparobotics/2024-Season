// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsytem extends SubsystemBase {
  private final CANSparkMax motorL = new CANSparkMax(ClimberConstants.MotorIDLeft, MotorType.kBrushless);
  private final CANSparkMax motorR = new CANSparkMax(ClimberConstants.MotorIDRight, MotorType.kBrushless);
  /** Creates a new ClimberSubsytem. */
  public ClimberSubsytem() {
    motorL.setInverted(true);
    motorR.setInverted(false);
  }
  public Command climb(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){
    return runOnce(() -> {
      motorL.set(leftSpeed.getAsDouble());
      motorR.set(rightSpeed.getAsDouble());

    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
