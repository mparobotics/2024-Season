// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
/*This class defines commands for running a Sysid Characterization routine for an arm 
powered by two Falcon500 motors and with angle measurements from a REV throughbore encoder.*/
public class ShooterSubsystem extends SubsystemBase {
  //Motor IDs
  final int ShooterID = 42;
  
  

  //Define motor
  private final CANSparkMax motor = new CANSparkMax(ShooterID, MotorType.kBrushless);
 

  
  
  private final RelativeEncoder encoder = motor.getEncoder();

  //A MutableMeausre contains a measurement of a physical quantity that can be updated with a new value each frame.
  // The units library is a bit annoying to use, but we're still using it because it handles all the unit conversions neatly.

  //define measurement variables for the voltage going to the motors, the arm's angle, and the arm's angular velocoity.
  private final MutableMeasure<Voltage> motor_voltage = MutableMeasure.ofBaseUnits(0,Units.Volts);

  private final MutableMeasure<Velocity<Angle>> motor_velocity = MutableMeasure.ofBaseUnits(0, Units.RadiansPerSecond);

 
  //use default configuration, but could potentially customize the voltages for the characterization routine by supplying them here
  private final SysIdRoutine.Config config = new SysIdRoutine.Config();


  private final Mechanism Arm = new Mechanism(
    (Measure<Voltage> volts) -> {runMotorsFromVoltage(volts);}, //code that runs the mechanism goes here. must use a Measure<Voltage> to supply voltage to the motors, 
                (SysIdRoutineLog log) -> logArmState(log), 
                this);

  private SysIdRoutine arm_sysid = new SysIdRoutine(config, Arm);
  
  

  public ShooterSubsystem() {
    motor.setIdleMode(IdleMode.kCoast);
    motor.setInverted(false);
    SmartDashboard.putData("Run Quasistatic Forward",arm_sysid.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Run Quasistatic Reverse",arm_sysid.quasistatic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Run Dynamic Forward",arm_sysid.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Run Dynamic Reverse",arm_sysid.dynamic(SysIdRoutine.Direction.kReverse));
    
  }
  
  public double getMotorVoltage(){
    return motor.get() * RobotController.getBatteryVoltage();
  }
  private void runMotorsFromVoltage(Measure<Voltage> volts){
    motor.setVoltage(volts.in(Units.Volts));
  }
  private void logArmState(SysIdRoutineLog log){
    log.motor("Arm Motors")
    .voltage(motor_voltage.mut_replace(Units.Volts.of(getMotorVoltage())))
    .angularVelocity(motor_velocity.mut_replace(Units.RadiansPerSecond.of(encoder.getVelocity())));
  }
  public Command controlArmWithJoystick(DoubleSupplier speed){
    return runOnce(() -> {
      if(Math.abs(speed.getAsDouble()) < 0.1){
        motor.set(0);
      }
      else{
        motor.set(speed.getAsDouble());
      }
      
    });
  }
  
  public Command stopMotor(){
    return runOnce(() -> motor.set(0));
  }

  @Override
  public void periodic() {
 

    SmartDashboard.putNumber("Arm Position ", encoder.getVelocity());

  }
}
