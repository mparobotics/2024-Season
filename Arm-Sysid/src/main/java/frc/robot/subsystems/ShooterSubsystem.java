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
  final int ShooterID = 21;
  final int IndexerID = 22;
  
  

  //Define motor
  private final CANSparkMax shooter = new CANSparkMax(ShooterID, MotorType.kBrushless);
  private final CANSparkMax indexer = new CANSparkMax(IndexerID, MotorType.kBrushless);

  private double voltage = 0;
  
  private final RelativeEncoder encoder = shooter.getEncoder();

  //A MutableMeausre contains a measurement of a physical quantity that can be updated with a new value each frame.
  // The units library is a bit annoying to use, but we're still using it because it handles all the unit conversions neatly.

  //define measurement variables for the voltage going to the motors, the arm's angle, and the arm's angular velocoity.
  private final MutableMeasure<Voltage> motor_voltage = MutableMeasure.ofBaseUnits(0,Units.Volts);

  private final MutableMeasure<Velocity<Angle>> motor_velocity = MutableMeasure.ofBaseUnits(0, Units.RotationsPerSecond);

  private final MutableMeasure<Angle> motor_position = MutableMeasure.ofBaseUnits(0, Units.Rotations);
  //use default configuration, but could potentially customize the voltages for the characterization routine by supplying them here
  private final SysIdRoutine.Config config = new SysIdRoutine.Config();


  private final Mechanism Arm = new Mechanism(
    (Measure<Voltage> volts) -> {runMotorsFromVoltage(volts);}, //code that runs the mechanism goes here. must use a Measure<Voltage> to supply voltage to the motors, 
                (SysIdRoutineLog log) -> logShooterState(log), 
                this);

  private SysIdRoutine shooter_sysid = new SysIdRoutine(config, Arm);
  
  

  public ShooterSubsystem() {
    shooter.setIdleMode(IdleMode.kCoast);
    shooter.setInverted(false);
    indexer.setInverted(true);
    SmartDashboard.putData("Shooter: Run Quasistatic Forward",shooter_sysid.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Shooter: Run Quasistatic Reverse",shooter_sysid.quasistatic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Shooter: Run Dynamic Forward",shooter_sysid.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Shooter: Run Dynamic Reverse",shooter_sysid.dynamic(SysIdRoutine.Direction.kReverse));
    
  }
  
  public double getMotorVoltage(){
    return voltage * RobotController.getBatteryVoltage();
  }
  private void runMotorsFromVoltage(Measure<Voltage> volts){
    voltage = volts.in(Units.Volts);
    shooter.setVoltage(voltage);
  }
  private void logShooterState(SysIdRoutineLog log){
    log.motor("Shooter Motor")
    .voltage(motor_voltage.mut_replace(Units.Volts.of(getMotorVoltage())))
    .angularVelocity(motor_velocity.mut_replace(Units.RotationsPerSecond.of(encoder.getVelocity()/60)))
    .angularPosition(motor_position.mut_replace(Units.Rotations.of(encoder.getPosition())));
  }
  public Command controlShooterWithJoystick(DoubleSupplier shootspeed, DoubleSupplier beltspeed){
    return runOnce(() -> {
      if(Math.abs(shootspeed.getAsDouble()) < 0.1){
        //shooter.set(0);
      }
      else{
        //shooter.set(shootspeed.getAsDouble());
      }
      if(Math.abs(beltspeed.getAsDouble()) < 0.1){
        indexer.set(0);
      }
      else{
        indexer.set(beltspeed.getAsDouble());
      }
      
    });
  }
  
  public Command stopMotor(){
    return runOnce(() -> shooter.set(0));
  }

  @Override
  public void periodic() {
 

    SmartDashboard.putNumber("Shooter Speed (RPMs)", encoder.getVelocity());

  }
}
