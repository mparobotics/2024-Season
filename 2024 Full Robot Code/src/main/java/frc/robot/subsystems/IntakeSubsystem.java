// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
  private final DigitalInput beamSensor = new DigitalInput(IntakeConstants.beamSensorPort);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }
  public void runIntake(double speed){
    intakeMotor.set(speed);
  }
  public boolean isNoteInIntake(){
    return beamSensor.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
