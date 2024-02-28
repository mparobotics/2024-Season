// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import java.util.function.DoubleSupplier;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  //two motors run in sync to drive the arm
  private final TalonFX motorR = new TalonFX(ArmConstants.RmotorID);
  private final TalonFX motorL = new TalonFX(ArmConstants.LmotorID);
  private final TalonFXConfiguration motorConfig= new TalonFXConfiguration();

  //a REV throughbore encoder tells us the arm's current angle
  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(1);


  //the arm uses a PID controller to automatically move to each setpoint
  private PIDController armPID = new PIDController(ArmConstants.kP,ArmConstants.kI, ArmConstants.kD);
  
  //the angle we want the arm to move to 
  private double setpoint = 20;



  //An InterpolatingTreeMap interpolates between measured data points to figure out what angle to aim the shooter based on how far away from the speaker we are
  private InterpolatingTreeMap<Double,Double> armAngleMap = new InterpolatingDoubleTreeMap();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    
    setTarget(getArmPosition());
    

    motorConfig.CurrentLimits.StatorCurrentLimit = 50;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorL.getConfigurator().apply(motorConfig);
    motorR.getConfigurator().apply(motorConfig);
    
    
    
    SmartDashboard.putData("Arm to handoff", setArmSetpointCommand(() -> ArmConstants.handoffPosition));
    SmartDashboard.putData("Arm to amp", setArmSetpointCommand(() -> ArmConstants.ampPosition));


    //The left motor follows the right motor. The right motor runs the control loop, and the left motor copies the output of the right motor
    //opposite motor directions spin the arm in the same direction, so opposeMasterDirection is set to true
    motorL.setControl(new Follower(ArmConstants.RmotorID, true));

    //populate the InterpolatingTreeMap with our data points
    for(Double[] dataPoint : ArmConstants.ArmAngleMapData){
      armAngleMap.put(dataPoint[0],dataPoint[1]);
    }

    SmartDashboard.putNumber("Arm Set Setpoint", setpoint);
  }
  //set the goal position to a specified angle. limit the setpoint to be between the max and min positions
  public void setTarget(double degrees){
    setpoint = Math.max(Math.min(degrees, ArmConstants.maxArmPosition), ArmConstants.minArmPosition);
  }
  //set the arm angle to score from a certain distance away
  public void setAngleForShootingDistance(double meters){
    setTarget(armAngleMap.get(meters));
  }
  //set the arm to score in the amp
  public void setToAmpAngle(){
    setTarget(ArmConstants.ampPosition);
  }
  //return the arm to the down position
  public void setToHandoffAngle(){
    setTarget(ArmConstants.handoffPosition);
  }


  //get the arm position in degrees
  public double getArmPosition(){
    return armEncoder.getAbsolutePosition() * 360 - 200.2 + 90;
  }
  //returns true if the arm is within 1 degree of the target postion
  public boolean isAtTarget(){
    return Math.abs(setpoint - getArmPosition()) < 1;
  }


  public Command setArmSetpointCommand(DoubleSupplier setpoint){
  return runOnce(() ->{setTarget(setpoint.getAsDouble());});
  }
  public Command teleopArmControlCommand(DoubleSupplier speed){
    return runOnce(() -> motorR.set(speed.getAsDouble()  * 0.4));
  }

  @Override
  public void periodic() {
    //The PID controller tries to minimize the difference between the goal position and the actual arm's position
    double PIDOutput = armPID.calculate(getArmPosition(),setpoint);

    SmartDashboard.putNumber("PID output", PIDOutput);
    //set the motor speed to the output of the PID controller
    motorR.set(-PIDOutput);

    SmartDashboard.putNumber("Arm Position", getArmPosition());
    SmartDashboard.putNumber("Arm Goal Position", setpoint);
    SmartDashboard.putBoolean("Is at setpoint", isAtTarget());
    double error = setpoint - getArmPosition();
    
   
    SmartDashboard.putNumber("Arm Error", error);
    
  }
}
