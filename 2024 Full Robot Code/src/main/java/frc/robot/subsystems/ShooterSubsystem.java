// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


import java.util.function.DoubleSupplier;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


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
  

  

  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    beltMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setIdleMode(IdleMode.kCoast);

    beltMotor.setInverted(true);

    shooterMotor.setSmartCurrentLimit(80);
    shooterEncoder.setVelocityConversionFactor(1);
  }
  
  public boolean isNoteInShooter(){
    return !beamSensor.get();
  }
  public double getShooterWheelSpeed(){
    return shooterEncoder.getVelocity();
  }
  public boolean isAtShootingSpeed(){
    return(getShooterWheelSpeed() > ShooterConstants.shooterWheelSpeed);
  }

  public void setBeltSpeed(double speed){
    beltMotor.set(speed);
  }
  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
  }
  public void spinUpShooter(){
    setShooterSpeed(1);
  }
  public void stopShooting(){
    beltMotor.set(0);
    shooterMotor.set(0);
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
