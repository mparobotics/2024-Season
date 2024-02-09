// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends TrapezoidProfileSubsystem {
  //two motors run in sync to drive the arm
  private final TalonFX motorR = new TalonFX(ArmConstants.RmotorID);
  private final TalonFX motorL = new TalonFX(ArmConstants.LmotorID);
  
  private final RelativeEncoder armEncoder = new CANSparkMax(ArmConstants.encoderID, MotorType.kBrushed).getEncoder();
  //configurations for the motors and control loop
  private PIDController armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  private ArmFeedforward armFF= new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  private double setpoint;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    
    
    super(new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity,ArmConstants.kMaxAcceleration));

    

    //The left motor follows the right motor. The right motor runs the control loop, and the left motor copies the output of the right motor
    //opposite motor directions spin the arm in the same direction, so opposeMasterDirection is set to true
    motorL.setControl(new Follower(ArmConstants.RmotorID, true));
    
  }
  //move the arm to the desired position
  public void setTarget(double degrees){
    //use the contoller defined in slot0
    setpoint = Units.degreesToRadians(degrees);
    setGoal(setpoint);
    
  }
  public double getArmPosition(){
    return armEncoder.getPosition() * ArmConstants.ticksToRadians;
  }
  public boolean isAtTarget(){
    return Math.abs(setpoint - getArmPosition()) < 0.01;
  }
  
  @Override
  protected void useState(State state) {
    motorR.setVoltage(armFF.calculate(state.position,state.velocity) + armPID.calculate(getArmPosition(),state.position));
  }
  @Override
  public void periodic() {
    
   
  }

  
}
