// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsytem extends SubsystemBase {
  private final CANSparkMax motorL = new CANSparkMax(ClimberConstants.MotorIDLeft, MotorType.kBrushless);
  private final CANSparkMax motorR = new CANSparkMax(ClimberConstants.MotorIDRight, MotorType.kBrushless);
  /** Creates a new ClimberSubsytem. */
  public ClimberSubsytem() {
    motorL.setInverted(false);
    motorR.setInverted(true);

    motorL.setIdleMode(IdleMode.kBrake);
    motorR.setIdleMode(IdleMode.kBrake);
  }
  /** a command to run the two climbers, with boolean inputs to control each arm's up/ down motion independently */
  public Command climb(BooleanSupplier leftUp, BooleanSupplier leftDown, BooleanSupplier rightUp, BooleanSupplier rightDown){
    return runOnce(() -> {
      double leftSpeed = (leftUp.getAsBoolean()? 1: (leftDown.getAsBoolean()? -1:0));
      double rightSpeed = (rightUp.getAsBoolean()? 1: (rightDown.getAsBoolean()? -1:0));
      motorL.set(leftSpeed);
      motorR.set(rightSpeed);

    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
