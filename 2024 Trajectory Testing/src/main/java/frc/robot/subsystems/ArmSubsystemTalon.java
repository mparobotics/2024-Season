// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystemTalon extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public double m_armKP = 1; //untested PID values
  public double m_armKI = 0;
  public double m_armKD = 0;

  public double m_armKG = 0; //G, V, S, and A are also untested
  public double m_armKV = 0; 
  public double m_armKS = 0;
  public double m_armKA = 0;

  public double armDown = 0;
  public double armUp = 0;

  private TalonFX armMotorL = new TalonFX(1);
  private TalonFX armMotorR = new TalonFX(2);

  private MotionMagicConfigs MMconfig = new MotionMagicConfigs();
  private SoftwareLimitSwitchConfigs limitSwtchConfig = new SoftwareLimitSwitchConfigs();
  private Slot0Configs PIDconfig = new Slot0Configs();
  
  public ArmSubsystemTalon() {
    SmartDashboard.putNumber("Arm Down Degrees", armDown);
    SmartDashboard.putNumber("Arm Up Degrees", armUp);
    SmartDashboard.putNumber("kP", m_armKP);
    SmartDashboard.putNumber("kI", m_armKI);
    SmartDashboard.putNumber("kD", m_armKD);


    armMotorR.getConfigurator().apply(PIDconfig);
    armMotorR.getConfigurator().apply(MMconfig);

    armMotorL.setControl(new Follower(2, true));
  }

  public Command downPosition()
  {
    return runOnce( 
      () -> {
        armMotorR.setControl(new MotionMagicVoltage(Units.degreesToRotations(armDown)));
      }
      );
    }

  public Command upPosition()
  {
    return runOnce( 
      () -> {
        armMotorR.setControl(new MotionMagicVoltage(Units.degreesToRotations(armUp)));
      }
      );
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //this code allows values to be changed from SmartDashboard
    armDown = SmartDashboard.getNumber("Arm Down Degrees", 0);
    armUp = SmartDashboard.getNumber("Arm Up Degrees", 0);
    m_armKP = SmartDashboard.getNumber("kP", 0);
    m_armKI = SmartDashboard.getNumber("kI", 0);
    m_armKD = SmartDashboard.getNumber("kD", 0);
  }
}
