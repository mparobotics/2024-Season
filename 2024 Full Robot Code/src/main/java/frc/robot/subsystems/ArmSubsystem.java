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
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;
//the arm uses a TrapezoidProfileSubsystem to automatically generate a smooth motion to each setpoint
public class ArmSubsystem extends TrapezoidProfileSubsystem {
  //two motors run in sync to drive the arm
  private final TalonFX motorR = new TalonFX(ArmConstants.RmotorID);
  private final TalonFX motorL = new TalonFX(ArmConstants.LmotorID);
  

  private final RelativeEncoder armEncoder = new CANSparkMax(ArmConstants.encoderID, MotorType.kBrushed).getEncoder();
  
  //A Feedforward and PID controller work together to bring the arm to its target position
  private PIDController armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  private ArmFeedforward armFF = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  private double setpoint;
  private double profilePosition;

  //An InterpolatingTreeMap interpolates between measured data points to determine the correct arm angle for a given distance
  private InterpolatingTreeMap<Double,Double> armAngleMap = new InterpolatingTreeMap<Double,Double>(null,null);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //Supply the motion profile generator with a maximum velocity and acceleration and an inital position of the arm
    super(new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity,ArmConstants.kMaxAcceleration), ArmConstants.handoffPosition);

    

    //The left motor follows the right motor. The right motor runs the control loop, and the left motor copies the output of the right motor
    //opposite motor directions spin the arm in the same direction, so opposeMasterDirection is set to true
    motorL.setControl(new Follower(ArmConstants.RmotorID, true));

    
    //populate the InterpolatingTreeMap with our data points
    for(Double[] dataPoint : ArmConstants.ArmAngleMapData){
      armAngleMap.put(dataPoint[0],dataPoint[1]);
    }
   
  }
  //set the goal position to a specified angle
  public void setTarget(double radians){
    setpoint = radians;
    setGoal(setpoint);
    
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


  public double getArmPosition(){
    return armEncoder.getPosition() * ArmConstants.ticksToRadians;
  }
  public boolean isAtTarget(){
    return Math.abs(setpoint - getArmPosition()) < 0.01;
  }
  
  @Override
  protected void useState(State state) {
    motorR.setVoltage(armFF.calculate(state.position,state.velocity) + armPID.calculate(getArmPosition(),state.position));
    profilePosition = state.position;
  }
  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Arm Position", getArmPosition());
    SmartDashboard.putNumber("Arm Goal Position", setpoint);
    SmartDashboard.putNumber("Arm Profile Position", profilePosition);
  }

  
}
