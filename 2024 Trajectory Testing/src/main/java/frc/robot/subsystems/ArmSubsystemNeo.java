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

public class ArmSubsystemNeo extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  public double m_neoArmKP = 1; //untested PID values
  public double m_neoArmKI = 0;
  public double m_neoArmKD = 0;

  public double armDown = 0;
  public double armUp = 0;

  private CANSparkFlex armMotor = new CANSparkFlex(21, MotorType.kBrushless);
  private SparkPIDController armController = armMotor.getPIDController();

  public ArmSubsystemNeo() {
    //SmartDashboard.putNumber("Arm Down", armDown);
    //SmartDashboard.putNumber("Arm Up", armUp);

    armController.setP(m_neoArmKP);
    armController.setI(m_neoArmKI);
    armController.setD(m_neoArmKD);
  }

  public Command neoDownPosition()
  {
    return runOnce( 
      () -> {
        armController.setReference(armDown, ControlType.kPosition);
      }
      );
    }

  public Command neoUpPosition()
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
    //armDown = SmartDashboard.getNumber("Arm Down", 0);
    //armUp = SmartDashboard.getNumber("Arm Up", 0);

  }
}
