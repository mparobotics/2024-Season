// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax beltMotor = new CANSparkMax(ShooterConstants.beltMotorID, MotorType.kBrushless);
  private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
  private final DigitalInput beamSensor = new DigitalInput(ShooterConstants.beamSensorPort);

  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
  private final SparkPIDController shooterSpeedController = shooterMotor.getPIDController();


  private double setpoint;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    beltMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setIdleMode(IdleMode.kCoast);

    beltMotor.setInverted(true);

    shooterMotor.setSmartCurrentLimit(80);
    shooterEncoder.setVelocityConversionFactor(1);

    shooterSpeedController.setP(ShooterConstants.kP);
    shooterSpeedController.setI(ShooterConstants.kI);
    shooterSpeedController.setD(ShooterConstants.kD);
    

    SmartDashboard.putData("Spin Up Shooter", spinUpShooterCommand());
  }
  public void setBeltMotorIdleMode(IdleMode mode){
    beltMotor.setIdleMode(mode);
  }
  public boolean isNoteInShooter(){
    return !beamSensor.get();
  }
  public double getShooterWheelSpeed(){
    return shooterEncoder.getVelocity();
  }
  public boolean isAtShootingSpeed(){
    return(Math.abs(getShooterWheelSpeed() - ShooterConstants.shooterWheelSpeed) < 1);
  }
  public void setBeltSpeed(double speed){
    beltMotor.set(speed);
  }
  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
  }
  public void spinUpShooter(){
    setTargetShooterSpeed(ShooterConstants.shooterWheelSpeed);
  }
  public void stopShooting(){
    beltMotor.set(0);
    shooterMotor.set(0);
  }
  public void setTargetShooterSpeed(double rpm){
    shooterSpeedController.setReference(rpm, ControlType.kVelocity, 0, rpm * ShooterConstants.kFF);
    setpoint = rpm;
  }
  
  public Command shooterControlCommand(DoubleSupplier shooterSpeed, DoubleSupplier beltSpeed){
    return runOnce(() ->{
      if(Math.abs(beltSpeed.getAsDouble()) > 0.1){
        setBeltSpeed(beltSpeed.getAsDouble());
      }
      else{
        setBeltSpeed(0);
      }
      setShooterSpeed(shooterSpeed.getAsDouble());
      
    });
  }
  public Command spinUpShooterCommand(){
    return runOnce(() -> {
      spinUpShooter();
    });
  }
  public Command stopShooterCommand(){
    return runOnce(()-> {
      stopShooting();
    });
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is Note In Shooter", isNoteInShooter());
    SmartDashboard.putNumber("shooter Speed", shooterEncoder.getVelocity());
    SmartDashboard.putBoolean("Is at shooting speed", isAtShootingSpeed());
  }
}
