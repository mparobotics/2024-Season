// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class ArmSubsystem extends SubsystemBase {
  final int RmotorID = 0;
  final int LmotorID = 0;

  private final TalonFX motorR = new TalonFX(RmotorID);
  private final TalonFX motorL = new TalonFX(LmotorID);
  private final RelativeEncoder encoder = new CANSparkMax(0,MotorType.kBrushed).getEncoder();

  private final MutableMeasure<Voltage> arm_motor_voltage = MutableMeasure.ofBaseUnits(0,Units.Volts);

  private final MutableMeasure<Angle> arm_position = MutableMeasure.ofBaseUnits(0,Units.Radians);

  private final MutableMeasure<Velocity<Angle>> arm_velocity = MutableMeasure.ofBaseUnits(0, Units.RadiansPerSecond);

  private final SysIdRoutine.Config config = new SysIdRoutine.Config();
  private final Mechanism Arm = new Mechanism(
    (Measure<Voltage> volts) -> {runMotorsFromVoltage(volts);}, //code that runs the mechanism goes here. must use a Measure<Voltage> to supply voltage to the motors, 
                (SysIdRoutineLog log) -> logArmState(log), 
                this);

  private SysIdRoutine arm_sysid = new SysIdRoutine(config, Arm);
  
  

  public ArmSubsystem() {
    motorL.setControl(new Follower(RmotorID, true));
  }
  public double getEncoderRadians(){
    return encoder.getPosition() * 2 * Math.PI / 8192;
  }
  public double getEncoderRadiansPerSecond(){
    return encoder.getVelocity();
  }
  public double getMotorVoltage(){
    return (motorR.get() + motorL.get())/2 * RobotController.getBatteryVoltage();
  }
  private void runMotorsFromVoltage(Measure<Voltage> volts){
    motorR.setVoltage(volts.in(Units.Volts));
  }
  private void logArmState(SysIdRoutineLog log){
    log.motor("Arm Motors")
    .voltage(arm_motor_voltage.mut_replace(Units.Volts.of(getMotorVoltage())))
    .angularPosition(arm_position.mut_replace(Units.Radians.of(getEncoderRadians())))
    .angularVelocity(arm_velocity.mut_replace(Units.RadiansPerSecond.of(getEncoderRadiansPerSecond())));
  }
  


  @Override
  public void periodic() {
    SmartDashboard.putData("Run Quasistatic Forward",arm_sysid.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Run Quasistatic Reverse",arm_sysid.quasistatic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Run Dynamic Forward",arm_sysid.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Run Dynamic Reverse",arm_sysid.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
