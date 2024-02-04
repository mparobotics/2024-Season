// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  //two motors run in sync to drive the arm
  private final TalonFX motorR = new TalonFX(ArmConstants.RmotorID);
  private final TalonFX motorL = new TalonFX(ArmConstants.LmotorID);
  

  //configurations for the motors and control loop
  private MotionMagicConfigs MMconfig = new MotionMagicConfigs();
  private SoftwareLimitSwitchConfigs limitConfig = new SoftwareLimitSwitchConfigs();
  private Slot0Configs PIDconfig = new Slot0Configs();

  double setpoint;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    limitConfig.ForwardSoftLimitEnable = true;
    limitConfig.ReverseSoftLimitEnable = true;

    limitConfig.ReverseSoftLimitThreshold = ArmConstants.maxArmPosition;
    limitConfig.ReverseSoftLimitThreshold = ArmConstants.minArmPosition;
    
    MMconfig.MotionMagicAcceleration = ArmConstants.kMaxAcceleration;
    MMconfig.MotionMagicCruiseVelocity = ArmConstants.kMaxVelocity;
    
    /*gravity affects the arm's motion differently depending on the arm's position.
    the force of gravity has the greatest effect when the arm is horizontal and zero effect when the arm is vertical
    this can be represented as the cosine of the arm's angle.*/
    PIDconfig.GravityType = GravityTypeValue.Arm_Cosine;

    /*The Arm is controlled using a combination of Feedforward control and PID control. 
    The feedforward estimates the voltage that should be supplied to the motors to get them 
    to move at a desired velocity and acceleration, however it isn't perfect, so we use a PID loop to correct for any error that builds up */
    
    //configure feedforward
    PIDconfig.kG = ArmConstants.kG;
    PIDconfig.kS = ArmConstants.kS;
    PIDconfig.kV = ArmConstants.kV;
    PIDconfig.kA = ArmConstants.kA;
    //configure PID
    PIDconfig.kP = ArmConstants.kP;
    PIDconfig.kI = ArmConstants.kI;
    PIDconfig.kD = ArmConstants.kD;

    motorR.getConfigurator().apply(PIDconfig);
    motorR.getConfigurator().apply(MMconfig);

    //The left motor follows the right motor. The right motor runs the control loop, and the left motor copies the output of the right motor
    //opposite motor directions spin the arm in the same direction, so opposeMasterDirection is set to true
    motorL.setControl(new Follower(ArmConstants.RmotorID, true));
    
  }
  //move the arm to the desired position
  public void setTarget(double degrees){
    motorR.setControl(new MotionMagicDutyCycle(Units.degreesToRotations(degrees)));
    setpoint = degrees;
  }
  public double getArmPosition(){
    return 0;
  }
  public boolean isAtTarget(){
    return setpoint - getArmPosition() < 0.01;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
