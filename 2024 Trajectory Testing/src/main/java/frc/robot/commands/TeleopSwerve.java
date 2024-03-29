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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.OnboardModuleState;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class TeleopSwerve extends Command {
  private SwerveSubsystem m_SwerveSubsystem;
  private DoubleSupplier m_xSupplier, m_ySupplier, m_rotationSupplier;
  private BooleanSupplier m_robotCentricSupplier, m_isAmpScoringSupplier, m_isSpeakerScoringSupplier;



  private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, SwerveConstants.autoAlignXYConstraints);
  private ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, SwerveConstants.autoAlignXYConstraints);
  private ProfiledPIDController angleController = new ProfiledPIDController(0, 0, 0, SwerveConstants.autoAlignRConstraints);


  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0); 
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(SwerveSubsystem SwerveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier,
      BooleanSupplier isAmpScoringSupplier,
      BooleanSupplier isSpeakerScoringSupplier
      ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);
    this.m_xSupplier = xSupplier;
    this.m_ySupplier = ySupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_robotCentricSupplier = robotCentricSupplier;
    this.m_isAmpScoringSupplier = isAmpScoringSupplier;
    this.m_isSpeakerScoringSupplier = isSpeakerScoringSupplier;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        /* Get Values, applies Deadband, (doesnt do anything if stick is less than a value)*/
    double xVal =
        xLimiter.calculate(
            MathUtil.applyDeadband(m_xSupplier.getAsDouble(), SwerveConstants.inputDeadband));
    double yVal =
        yLimiter.calculate(
            MathUtil.applyDeadband(m_ySupplier.getAsDouble(), SwerveConstants.inputDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), SwerveConstants.inputDeadband));
    boolean isFieldOriented = !m_robotCentricSupplier.getAsBoolean();
    boolean isAmpScoring = m_isAmpScoringSupplier.getAsBoolean();
    boolean isSpeakerScoring = m_isSpeakerScoringSupplier.getAsBoolean();

    boolean isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d currentPose = m_SwerveSubsystem.getPose();
    double currentDirection = currentPose.getRotation().getDegrees();


    
    if(isAmpScoring){
      //If we are trying to line up with the amp, override all drive inputs and move towards the scoring pose
      Pose2d targetPose = isRedAlliance? FieldConstants.RED_AMP_SCORING: FieldConstants.BLUE_AMP_SCORING;
      
      
      double targetDirection = OnboardModuleState.smolOptimize180(currentDirection,targetPose.getRotation().getDegrees());

      double xSpeed = xController.calculate(currentPose.getX(),targetPose.getX());
      double ySpeed = yController.calculate(currentPose.getY(),targetPose.getY());
      double rSpeed = angleController.calculate(currentDirection, targetDirection);
      
      m_SwerveSubsystem.drive(xSpeed, ySpeed, rSpeed, true);
    }
    else if(isSpeakerScoring){
      //if we are trying to aim at the speaker, override the rotation command and rotate towards the scoring direction, but keep the translation commands
        Translation2d targetLocation = isRedAlliance? FieldConstants.RED_SPEAKER_LOCATION: FieldConstants.BLUE_SPEAKER_LOCATION;
        Translation2d relativeTargetPosition = targetLocation.minus(currentPose.getTranslation());

        double targetDirection = OnboardModuleState.smolOptimize180(currentDirection, relativeTargetPosition.getAngle().getDegrees());
        double rSpeed = angleController.calculate(currentDirection, targetDirection);
        m_SwerveSubsystem.drive(
        //the joystick values (-1 to 1) multiplied by the max speed of the drivetrain
      xVal * SwerveConstants.maxSpeed, 
      yVal * SwerveConstants.maxSpeed, 
      rSpeed, 
      isFieldOriented);
    }
    else{
      //Just drive normally
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
