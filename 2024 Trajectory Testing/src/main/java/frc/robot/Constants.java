// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveModuleConstants;


public final class Constants {
  public static final class SwerveConstants{
    public static final double inputDeadband = .1;
    public static final int PIGEON_ID = 17; 
    public static final boolean invertPigeon = false;

    /* Drivetrain Constants */
    public static final double halfTrackWidth = Units.inchesToMeters(17.5) / 2;
    public static final double halfWheelBase = Units.inchesToMeters(20.5) / 2;
    public static final double driveBaseRadius =  Math.sqrt(halfWheelBase * halfWheelBase + halfTrackWidth * halfTrackWidth);

    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1 L2 Mk4 Modules
    //L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1 MK4 SDS Modules

    //give location of each module to a swerveDriveKinematics relative to robot center in meters
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(halfWheelBase, halfTrackWidth), 
      new Translation2d(halfWheelBase, -halfTrackWidth),
      new Translation2d(-halfWheelBase, halfTrackWidth),
      new Translation2d(-halfWheelBase, -halfTrackWidth)
    );
    //Config for PathPlanner. contains trajectory PID constants and other drivebase data
    public static final HolonomicPathFollowerConfig pathConfig = new HolonomicPathFollowerConfig( 
      new PIDConstants(5.0, 0.00001, 0.0), // Translation PID constants
      new PIDConstants(5.0, 0.0005, 0.001), // Rotation PID constants
      4.5, // Max module speed, in m/s
      driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;
       
    //Swerve Current Limiting for neos
    public static final int angleContinuousCurrentLimit = 20; //limits current draw of turning motor
    public static final int driveContinuousCurrentLimit = 50; //limits current draw of drive motor
  


    /* Drive Motor PID Values */
    public static final double driveKP = 0.1; //to tune
    public static final double driveKI = 0.0; //to tune
    public static final double driveKD = 0.0; //to tune
   public static final double driveKFF = 0.0; //to tune

    /* Drive Motor Characterization Values */
    //values to calculate the drive feedforward (KFF)
    public static final double driveKS = 0.667; //to calculate
    public static final double driveKV = 2.44; //to calculate
    public static final double driveKA = 0.27; //to calculate

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
    (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 9; // meters per second
    public static final double maxAngularVelocity = 11.5; //radians per second

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;
    

        /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1; 
      public static final int angleMotorID = 2; 
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(342.1-180);
    /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(330.5-180);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
        
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(223.2-180);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
  
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
        //creates a constant with all info from swerve module
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 12 ;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(320.8-180);
        /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }
  

    public static final boolean angleMotorInvert = false;
    public static final boolean driveMotorInvert = false;

    



  }




}
