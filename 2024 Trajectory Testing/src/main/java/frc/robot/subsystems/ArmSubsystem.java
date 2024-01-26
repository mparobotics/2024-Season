// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  public double m_armKP = 1; //untested PID values
  public double m_armKI = 0;
  public double m_armKD = 0;

  public double armDown = 0;
  public double armUp = 0;

  private CANSparkFlex armMotor = new CANSparkFlex(21, MotorType.kBrushless);
  private SparkPIDController armController = armMotor.getPIDController();

  public ArmSubsystem() {
    SmartDashboard.putNumber("Arm Down", armDown);
    SmartDashboard.putNumber("Arm Up", armUp);

    armController.setP(m_armKP);
    armController.setI(m_armKI);
    armController.setD(m_armKD);
  }

  public Command downPosition()
  {
    return runOnce( 
      () -> {
        armController.setReference(armDown, ControlType.kPosition);
      }
      );
    }

  public Command upPosition()
  {
    return runOnce( 
      () -> {
        armController.setReference(armUp, ControlType.kPosition);
      }
      );
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //this code allows values to be changed from SmartDashboard
    armDown = SmartDashboard.getNumber("Arm Down", 0);
    armUp = SmartDashboard.getNumber("Arm Up", 0);

  }
}
