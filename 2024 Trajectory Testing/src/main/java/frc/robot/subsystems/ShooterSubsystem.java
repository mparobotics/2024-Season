// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax beltMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax shooterMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final DigitalInput beamSensor = new DigitalInput(ShooterConstants.beamSensorPort);

  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
  private final SparkPIDController shooterSpeedController = shooterMotor.getPIDController();
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    beltMotor.setIdleMode(IdleMode.kBrake);
    shooterMotor.setIdleMode(IdleMode.kCoast);

    shooterSpeedController.setP(1);
    shooterSpeedController.setI(0);
    shooterSpeedController.setD(0);
  }

  public boolean isNoteInShooter(){
    return beamSensor.get();
  }
  public double getShooterWheelSpeed(){
    return shooterEncoder.getVelocity();
  }
  public void setTargetShooterSpeed(double rotationsPerSec){
    shooterSpeedController.setReference(rotationsPerSec, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
