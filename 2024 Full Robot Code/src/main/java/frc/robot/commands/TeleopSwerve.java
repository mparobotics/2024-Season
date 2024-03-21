// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.OnboardModuleState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;



public class TeleopSwerve extends Command {
  private SwerveSubsystem m_SwerveSubsystem;
  private DoubleSupplier m_xSupplier, m_ySupplier, m_rotationSupplier;
  private BooleanSupplier m_robotCentricSupplier, m_isSpeakerScoringSupplier;


  private ProfiledPIDController angleController = new ProfiledPIDController(0.2, 0, 0, AutoConstants.autoAlignRConstraints);


  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0); 
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  /** Main driving command for controlling the swerve drive, with options to switch between robot- and field- centric driving, as well as enabling auto aiming at the speaker*/
  public TeleopSwerve(SwerveSubsystem SwerveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier,
      BooleanSupplier isSpeakerScoringSupplier
      ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);
    this.m_xSupplier = xSupplier;
    this.m_ySupplier = ySupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_robotCentricSupplier = robotCentricSupplier;
    this.m_isSpeakerScoringSupplier = isSpeakerScoringSupplier;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if we're on red alliance, translation values are inverted since the drivers face the opposite direction
    int invert = FieldConstants.isRedAlliance()? -1: 1;
    
    double xVal = invert * 
        xLimiter.calculate(
            MathUtil.applyDeadband(m_xSupplier.getAsDouble(), SwerveConstants.inputDeadband));
    double yVal = invert * 
        yLimiter.calculate(
            MathUtil.applyDeadband(m_ySupplier.getAsDouble(), SwerveConstants.inputDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), SwerveConstants.inputDeadband));
    
    boolean isFieldOriented = !m_robotCentricSupplier.getAsBoolean();


    
    boolean isSpeakerScoring = m_isSpeakerScoringSupplier.getAsBoolean();
   
    

    double currentDirection = m_SwerveSubsystem.getPose().getRotation().getDegrees();
    //calculate the X and Y offsets from the robot to the speaker in meters
    Translation2d relativeTargetPosition = m_SwerveSubsystem.getRelativeSpeakerLocation(); 

    //calculate the fastest way to reach that angle (if we're at 355 degrees but we want to be at 5 degrees, its much better to rotate in the positive direction than to go all the way back around) 
    double targetDirection = OnboardModuleState.smolOptimize180(currentDirection, relativeTargetPosition.getAngle().getDegrees() + 180);
    SmartDashboard.putNumber("Speaker Direction", targetDirection);
    SmartDashboard.putNumber("Speaker Distance", relativeTargetPosition.getNorm());

    
    //plug the target angle into a PID controller, which will output a speed that can be supplied to the drivetrain
    double rSpeed = angleController.calculate(currentDirection, targetDirection);


    
  
    if(isSpeakerScoring){
      //If the driver is pressing the auto align button, then we override the rotation input from the controller with the PID controller which aims us at the speaker
        m_SwerveSubsystem.drive(
       
      xVal * SwerveConstants.maxSpeed, 
      yVal * SwerveConstants.maxSpeed, 
      rSpeed, 
      isFieldOriented);
    }
    else{
      //If we're not auto aiming, just drive normally
      /* Drive */
      m_SwerveSubsystem.drive(
      //the joystick values (-1 to 1) multiplied by the max speed of the drivetrain
      xVal * SwerveConstants.maxSpeed, 
      yVal * SwerveConstants.maxSpeed, 
      rotationVal * SwerveConstants.maxAngularVelocity, 
      isFieldOriented);
    }
  }
}
