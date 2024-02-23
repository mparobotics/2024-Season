// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
//the arm uses a TrapezoidProfileSubsystem to automatically generate a smooth motion to each setpoint
public class ArmSubsystem extends SubsystemBase {
  //two motors run in sync to drive the arm
  private final TalonFX motorR = new TalonFX(ArmConstants.RmotorID);
  private final TalonFX motorL = new TalonFX(ArmConstants.LmotorID);

  private final TalonFXConfiguration motorConfig= new TalonFXConfiguration();
  //private final RelativeEncoder armEncoder = new CANSparkMax(ArmConstants.encoderID, MotorType.kBrushless).getEncoder();
  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(1);
  //A Feedforward controller moves the arm according to the motion profile, and the PID controller corrects for any error in the system
  //These constants will be calculated with a SysID test
  private PIDController armPID = new PIDController(ArmConstants.kP,ArmConstants.kI, ArmConstants.kD);
  private ArmFeedforward armFF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  //The arm follows a trapezoidal profile to get to its target position. It will gradually speed up until it reaches the max velocity, and gradually slow down when it gets close to the target position
  private TrapezoidProfile profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity,ArmConstants.kMaxAcceleration));

  //The position we want to end up at
  private TrapezoidProfile.State goalState;

  //The position that the profile is currently at. This isn't necessarily the arm's true position, but the goal is to have them be as close as possible.
  private TrapezoidProfile.State currentState;
 

  //An InterpolatingTreeMap interpolates between measured data points to figure out what angle to aim the shooter based on how far away from the speaker we are
  private InterpolatingTreeMap<Double,Double> armAngleMap = new InterpolatingDoubleTreeMap();
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //Start by setting the goal state to the current position of the arm so the arm doesn't try to move on enable
    currentState = new TrapezoidProfile.State(getArmPosition(),0);
    goalState = currentState;

    motorConfig.CurrentLimits.StatorCurrentLimit = 50;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorL.getConfigurator().apply(motorConfig);
    motorR.getConfigurator().apply(motorConfig);
    armEncoder.setDistancePerRotation(360);
    
    SmartDashboard.putData("Zero Arm Enocder", runOnce(() -> motorR.setPosition(0)));
    SmartDashboard.putData("Arm to 0", setArmSetpointCommand(() -> 0));
    SmartDashboard.putData("Arm to 45", setArmSetpointCommand(() -> 45));

    //The left motor follows the right motor. The right motor runs the control loop, and the left motor copies the output of the right motor
    //opposite motor directions spin the arm in the same direction, so opposeMasterDirection is set to true
    motorL.setControl(new Follower(ArmConstants.RmotorID, true));

    

    //populate the InterpolatingTreeMap with our data points
    for(Double[] dataPoint : ArmConstants.ArmAngleMapData){
      armAngleMap.put(dataPoint[0],dataPoint[1]);
    }

    SmartDashboard.putNumber("Set Setpoint", 0);
  }
  //set the goal position to a specified angle
  public void setTarget(double degrees){
    goalState = new TrapezoidProfile.State(degrees, 0);
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
    return armEncoder.getDistance();
  }
  //returns true if the arm is close enough to the goal position
  public boolean isAtTarget(){
    return Math.abs(goalState.position - getArmPosition()) < 1;
  }
  
  
  public Command setArmSetpointCommand(DoubleSupplier setpoint){
  return runOnce(() ->{setTarget(setpoint.getAsDouble());});
  }
  public Command teleopArmControlCommand(DoubleSupplier speed){
    return runOnce(() -> motorR.set(speed.getAsDouble()  * 0.4));
  }
  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Arm Position", getArmPosition());
    SmartDashboard.putNumber("Arm Goal Position", goalState.position);

    //Move the current position 0.02 seconds farther along the profile
    currentState = profile.calculate(0.02,currentState,goalState);

    //The PID controller tries to minimize the difference between the profile's position and the actual arm's position
    double pid =  armPID.calculate(getArmPosition(),currentState.position);
    //the feedforward estimates the voltage the motor needs to be moving along the trajectory at the right speed
    double feedforward = armFF.calculate(Units.degreesToRadians(currentState.position), Units.degreesToRadians(currentState.velocity));

    double error = currentState.position - getArmPosition();
    SmartDashboard.putNumber("Arm Profile Position", currentState.position);
    SmartDashboard.putNumber("Arm Profile Velocity", currentState.velocity);
    SmartDashboard.putNumber("PID Correction", pid);
    SmartDashboard.putNumber("Arm Feedforward", pid);
    SmartDashboard.putNumber("Error", error);

    //Combine the feedforward and pid outputs and send them to the motor
    motorR.setVoltage(feedforward + pid);
    
    setTarget(SmartDashboard.getNumber("Set Setpoint", 0));
  }

  
}
