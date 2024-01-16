// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//A Subsystem to control a single NEO motor
public class MotorSubsystem extends SubsystemBase {

  //Create a SparkMAX motor controller
  private final CANSparkFlex testMotorL = new CANSparkFlex(11, MotorType.kBrushless);
  private final CANSparkMax testMotorR = new CANSparkMax(12, MotorType.kBrushless);
  
 
  public double shootSpeed = 1;

  //Sets up the PigeonIMU
  public WPI_Pigeon2 testPigeon = new WPI_Pigeon2(1);

  //get the pid controller from the motor
  private  SparkPIDController pid = testMotorL.getPIDController();

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    //set PID values
    pid.setP(1);
    pid.setI(0);
    pid.setD(0);

    //inverts right motor
    testMotorR.setInverted(true);


    SmartDashboard.putNumber("Shooting Speed", shootSpeed);


    
  }
  //A command that sets the motor to a given speed
  public CommandBase setMotor(DoubleSupplier speed){
    return runOnce(() -> {testMotorL.set(speed.getAsDouble()); testMotorR.set(speed.getAsDouble());});
  }

  //A command that sets the motor's setpoint to a specified angle and uses PID position control to get the motor to the target direction
  public CommandBase setPositionPID(DoubleSupplier position){
    return runOnce(() -> pid.setReference(position.getAsDouble(),ControlType.kPosition));
  }

  public CommandBase shoot(){
    double shootSpeed = SmartDashboard.getNumber("Shooting Speed", 0);
    return runOnce(() -> {testMotorL.set(shootSpeed); testMotorR.set(-shootSpeed);});
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pigeon Roll", testPigeon.getRoll());
  }
}
