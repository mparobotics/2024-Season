// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveModuleConstants;


/** We define all CAN IDs, pin numbers, motion control constants, field coordinates, etc. in this {@link Constants} file
All positions and lengths are in meters, and 
all rotations are in degrees unless otherwise noted.

Our localization systems assume that the far right corner of the blue alliance side of the field (where the red source station is)
is the origin. +X is towards the red alliance wall and +Y is to the left (from the perspective of the blue side).
0° is in the +X direction and positive rotations go counterclockwise.

  */
public final class Constants {
  public static final class IntakeConstants{
    public static final int intakeMotorID = 0;
    public static final int beamSensorPort = 0;
  }
  public static final class ShooterConstants{
    public static final int beltMotorID = 0;
    public static final int shooterMotorID = 0;
    public static final int beamSensorPort = 0;

    public static final double shooterWheelSpeed = 10; //RPMs
    
    
  }
  public static final class ArmConstants{
    public static final int LmotorID = 0;
    public static final int RmotorID = 0;
    public static final int encoderID = 0;

    public static final double minArmPosition = 0;
    public static final double maxArmPosition = 0;

    public static final double handoffPosition = 0;
    public static final double ampPosition = 0;

    public static final double kMaxAcceleration = 0; //max angular acceleration of the arm (rotations per second^2)
    public static final double kMaxVelocity = 0; //max angular velocity of the arm (rotations per second)

    //PID constants for the arm's motion control
    public static final double kP = 0; 
    public static final double kI = 0;
    public static final double kD = 0;


    //Feedforward constants for the arm's motion control
    public static final double kG = 0; //counteracts the force of gravity on the arm
    public static final double kS = 0; //counteracts friction in the mechanism
    public static final double kV = 0; //how much voltage to move a specific speed
    public static final double kA = 0; //how much voltage to accelerate a certain amount
    



  }
  public static final class FieldConstants{
    public static final Pose2d RED_AMP_SCORING = new Pose2d(0,0,Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_AMP_SCORING = new Pose2d(0,0,Rotation2d.fromDegrees(0));

    //location of the speaker target on the field in meters
    public static final Translation2d RED_SPEAKER_LOCATION = new Translation2d(0,0);
    public static final Translation2d BLUE_SPEAKER_LOCATION = new Translation2d(0,0);
  }
  public static final class VisionConstants{
    //offset from center of robot to the note-detector limelight (meters)
    public static final double noteLimelightTx = 0;
    public static final double noteLimelightTy = 0;
    public static final double noteLimelightTz = 0;

    public static final double noteLimelightAngle = 0; //degrees above horizontal
    
    //standard deviations of vision-based pose estimates
    public static final double STDDEV_X = 0.1; // meters
    public static final double STDDEV_Y = 0.1; 
    public static final double STDDEV_ROTATION = 5; //degrees



  }
  public static final class ClimberConstants{
    public static final int MotorIDLeft = 0;
    public static final int MotorIDRight = 0;
  }
  public static final class SwerveConstants{
    public static final double inputDeadband = .1;
    public static final int PIGEON_ID = 17; 
    public static final boolean invertPigeon = true;

    /* Drivetrain Constants */
    public static final double halfTrackWidth = Units.inchesToMeters(21) / 2; //half of the left-right distance between the wheels
    public static final double halfWheelBase = Units.inchesToMeters(21) / 2; //half of the forward-backward distance between the wheels
    public static final double driveBaseRadius =  Math.sqrt(halfWheelBase * halfWheelBase + halfTrackWidth * halfTrackWidth);

    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    

    public static final double driveGearRatio = (8.14 / 1.0); // 8.14:1 ( SDS Mk4 L1 Module )
    //L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1 ( SDS Mk4 L1 Module ) 
    //SDS Mk4 is 12.8:1,  Mk4i is 21.4:1
    

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;
 
    /* Maximum speed and angular velocity of the robot */
    public static final double maxSpeed = 9; // meters per second
    public static final double maxAngularVelocity = 11.5; //radians per second

    /*maximum speed and angular velocity while auto-aligning to a target */
    public static final double maxVelocityAutoAlign = 0; //   m/s
    public static final double maxAccelerationAutoAlign = 0;//   m/s^2
    public static final double maxAngularVelocityAutoAlign = 0;//   rad/s
    public static final double maxAngularAccelerationAutoAlign = 0;// rad/s^2

    /*maximum speeds during auto */
    public static final double maxVelocityAuto = 4; //  m/s
    public static final double maxAccelerationAuto = 3; //  m/s^2
    public static final double maxAngularVelocityAuto = 2 * Math.PI; //  rad/s
    public static final double maxAngularAccelerationAuto = 4 * Math.PI; //  rad/s^2

    public static final PathConstraints autoConstraints = new PathConstraints(maxVelocityAuto, maxAccelerationAuto, maxAngularVelocityAuto, maxAngularAccelerationAuto);
    public static final TrapezoidProfile.Constraints autoAlignXYConstraints = new TrapezoidProfile.Constraints(maxVelocityAutoAlign,maxAccelerationAutoAlign);
    public static final TrapezoidProfile.Constraints autoAlignRConstraints = new TrapezoidProfile.Constraints(maxAngularVelocityAutoAlign,maxAngularAccelerationAutoAlign);
    //give location of each module to a swerveDriveKinematics relative to robot center in meters
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(halfWheelBase, halfTrackWidth), 
      new Translation2d(halfWheelBase, -halfTrackWidth),
      new Translation2d(-halfWheelBase, -halfTrackWidth),
      new Translation2d(-halfWheelBase, halfTrackWidth)
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
      public static final int driveMotorID = 1; 
      public static final int angleMotorID = 2; 
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(342.1-180);
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          //creates a constant with all info from swerve module
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(330.5-180);
      
        
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          //creates a constant with all info from swerve module
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(223.2-180);
      
  
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        //creates a constant with all info from swerve module
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12 ;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(320.8-180);
       
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          //creates a constant with all info from swerve module
    }
  

    

  }
}
