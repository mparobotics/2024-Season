// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.SwerveModuleConstants;


/** We define all CAN IDs, pin numbers, motion control constants, field coordinates, etc. in this {@link Constants} file.


Our localization systems assume that the far right corner of the blue alliance side of the field (where the red source station is)
is the origin. +X is towards the red alliance wall and +Y is to the left (from the perspective of the blue side).
0° is in the +X direction and positive rotations go counterclockwise.

  */
public final class Constants {
  public static final class IntakeConstants{
    //A NEO vortex powers the intake rollers
    public static final int intakeMotorID = 20;
    //We originally planned to have a second beam break sensor on the intake, but it was never actually added
    public static final int beamSensorPort = 3;
  }
  public static final class ShooterConstants{
    //two neo motors, one for the indexer belts and one for the shooter wheels
    public static final int beltMotorID = 22;
    public static final int shooterMotorID = 21;
    //a beam break sensor allows us to detect if a note is in the shooter
    public static final int beamSensorPort = 0;
    //if we ever add shooting while moving, this will help us calculate the correct angle to aim
    public static final double noteSpeedMetersPerSecond = 20; //the speed that the note exits the shooter at

    //how fast does the shooter need to spin before we can shoot the note
    public static final double shooterWheelSpeed = 3500; //RPMs

    public static final double shootTimeSeconds = 0.1; //time to run the shooter for after the note is no longer detected. this is to prevent the wheels slowing down while still in contact with the note.

    //PID constants for controlling the shooter speed with closed loop control. we currently just run it at full speed all the time, but this could potentially give us more consistent shots
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 12.0/4500.0;

    
  }
  public static final class ArmConstants{
    //two falcon 500 motors work in unison to move the arm
    public static final int LmotorID = 41;
    public static final int RmotorID = 42;
    //a REV throughbore encoder measures the arm angle
    public static final int encoderPort = 1;
    
    //we limit the arm's position so it doesn't slam into the intake or extend past 12in of the frame perimeter
    public static final double minArmPosition = 19;
    public static final double maxArmPosition = 99;

    //arm angle for handing notes from the intake to the indexer
    public static final double handoffPosition = 19;
    //arm angle for scoring in the amp
    public static final double ampPosition = 98;

    //PID constants for the arm's motion control (all of our PID controllers only use P since they work fine without worrying about I or D)
    public static final double kP = 0.04; 
    public static final double kI = 0;
    public static final double kD = 0;

    

    public static final Double[][] ArmAngleMapData = {
    // each pair of doubles pairs a speaker distance with the ideal arm angle for that distance. 
    //These values are determined by doing physical testing with the real robot at a practice field.
    //During matches, we use pose estimation to calculate the distance to the speaker, and look it up in this table
    //We can then interpolate between these data points to approximate a good shooting angle for any distance in between the data points
    // the data is in the format: { DistanceToSpeaker (meters), Arm Angle(degrees) }
    
      {1.25,25.0},
      {1.48,32.0},
      {1.93,37.7},
      {2.38,42.0},
      {3.10,49.0}, 
    
    };
    public static final double maxShootingDistance = 3.10;
    

  }
  //these constants describe useful aspects of the field
  public static final class FieldConstants{
    public static final double FIELD_LENGTH = 16.4846; //distance from red side to blue side in meters
    public static final double FIELD_WIDTH = 8.1026; //distance from the amp side to the source side in meters
    //while not technically a constant, the alliance color is used so frequently, we put it in constants so it can be easily accessed from anywhere in the code
    public static boolean isRedAlliance(){
      return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }
    //this function allows us to mirror a pose to the correct side of the field to match our alliance color
    public static Pose2d flipPoseForAlliance(Pose2d pose){
      return isRedAlliance()? new Pose2d( FIELD_LENGTH - pose.getX(), pose.getY(), Rotation2d.fromDegrees(180).minus(pose.getRotation())): pose;
    }
    //the ideal pose for scoring in the amp
    public static final Pose2d BLUE_AMP_SCORING = new Pose2d(1.83,7.57,Rotation2d.fromDegrees(-90));
    public static final Pose2d RED_AMP_SCORING = new Pose2d(FIELD_LENGTH - BLUE_AMP_SCORING.getX(), BLUE_AMP_SCORING.getY(), Rotation2d.fromDegrees(180).minus(BLUE_AMP_SCORING.getRotation()));

    //location of the speaker target on the field in meters
    public static final Translation2d RED_SPEAKER_LOCATION = new Translation2d(FIELD_LENGTH,5.56);
    public static final Translation2d BLUE_SPEAKER_LOCATION = new Translation2d(0,5.56);
  }
  public static final class VisionConstants{
    //offset from center of robot to the note-detector limelight (meters)
    public static final double noteLimelightTx = 0;
    public static final double noteLimelightTy = 0;
    public static final double noteLimelightTz = 0;

    public static final double noteLimelightAngle = 0; //degrees above horizontal
    
  }
  public static final class ClimberConstants{
    public static final int MotorIDLeft = 32;
    public static final int MotorIDRight = 31;
  }
  public static final class AutoConstants{
    //Config for PathPlanner. contains trajectory PID constants and other drivebase data
    public static final HolonomicPathFollowerConfig pathConfig = new HolonomicPathFollowerConfig( 
        new PIDConstants(5.0, 0.00001, 0.0), // Translation PID constants for path following
        new PIDConstants(5.0, 0.0005, 0.001), // Rotation PID constants for path following
        SwerveConstants.maxSpeed, // Max module speed, in m/s
        SwerveConstants.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the Pathplanner API for the options here
    );

    /*maximum speed and angular velocity while auto-aligning to a target */
    public static final double maxVelocityAutoAlign = 3; //   m/s
    public static final double maxAccelerationAutoAlign = 3;//   m/s^2
    public static final double maxAngularVelocityAutoAlign = 0;//   rad/s
    public static final double maxAngularAccelerationAutoAlign = 0;// rad/s^2

    public static final double rotation_kP = 0.2;
    public static final double rotation_kI = 0;
    public static final double rotation_kD = 0;

    public static final double translation_kP = 1;
    public static final double translation_kI = 0;
    public static final double translation_kD = 0;
    
    public static final TrapezoidProfile.Constraints autoAlignXYConstraints = new TrapezoidProfile.Constraints(maxVelocityAutoAlign,maxAccelerationAutoAlign);
    public static final TrapezoidProfile.Constraints autoAlignRConstraints = new TrapezoidProfile.Constraints(maxAngularVelocityAutoAlign,maxAngularAccelerationAutoAlign);
    
  }
  public static final class SwerveConstants{
    public static final double inputDeadband = .1;
    public static final int PIGEON_ID = 17; 
    public static final boolean invertPigeon = true;

    /* Drivetrain Constants */
    public static final double halfTrackWidth = Units.inchesToMeters(21.0 / 2.0); //half of the left-right distance between the wheels
    public static final double halfWheelBase = Units.inchesToMeters(21.0 /2.0 ) ; //half of the forward-backward distance between the wheels
    public static final double driveBaseRadius =  Math.hypot(halfTrackWidth,halfWheelBase);

    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    

    public static final double driveGearRatio = (8.14 / 1.0); // 8.14:1 ( SDS Mk4 L1 Module )
    //L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1 ( SDS Mk4 L1 Module ) 
    //SDS Mk4 is 12.8:1,  Mk4i is 21.4:1
    

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelCircumference) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;
 
    /* Maximum speed and angular velocity of the robot */
    public static final double maxSpeed = 5; // meters per second
    public static final double maxAngularVelocity = maxSpeed / driveBaseRadius; //radians per second

    //give location of each module to a swerveDriveKinematics relative to robot center in meters
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(halfWheelBase, halfTrackWidth), 
      new Translation2d(-halfWheelBase, halfTrackWidth),
      new Translation2d(-halfWheelBase, -halfTrackWidth),
      new Translation2d(halfWheelBase, -halfTrackWidth)
    );
    

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;
       
    //Swerve Current Limiting for NEOs
    public static final int angleContinuousCurrentLimit = 20; //limits current draw of turning motor
    public static final int driveContinuousCurrentLimit = 50; //limits current draw of drive motor
  


    /* Drive Motor PID Values */
    public static final double driveKP = 0.1; 
    public static final double driveKI = 0.0; 
    public static final double driveKD = 0.0; 
   public static final double driveKFF = 0.0; 

    /* Drive Motor Feedforward Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44; 
    public static final double driveKA = 0.27; 

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01; 
    public static final double angleKI = 0.0; 
    public static final double angleKD = 0.0; 
    public static final double angleKFF = 0.0; 
  

   

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;


    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;
    
    public static final boolean angleMotorInvert = false;
    public static final boolean driveMotorInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 3; 
      public static final int angleMotorID = 2; 
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.2);
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          //creates a constant with all info from swerve module
    }

    /* Back Left Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(117.2);
      
        
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          //creates a constant with all info from swerve module
    }

    /* Back Right Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(141);
      
  
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        //creates a constant with all info from swerve module
    }

    /* Front Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 14 ;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-138);
       
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          //creates a constant with all info from swerve module
    }
  

    

  }
}
