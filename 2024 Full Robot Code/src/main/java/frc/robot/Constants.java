/* Copyright (c) FIRST and other WPILib contributors.// Open Source Software; you can modify and/or share it under the terms of// the WPILib BSD license file in the root directory of this project.*/package frc.robot;import com.pathplanner.lib.util.HolonomicPathFollowerConfig;import com.pathplanner.lib.util.PIDConstants;import com.pathplanner.lib.util.ReplanningConfig;import com.revrobotics.CANSparkBase.IdleMode;import edu.wpi.first.math.geometry.Pose2d;import edu.wpi.first.math.geometry.Rotation2d;import edu.wpi.first.math.geometry.Translation2d;import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;import edu.wpi.first.math.interpolation.InterpolatingTreeMap;import edu.wpi.first.math.kinematics.SwerveDriveKinematics;import edu.wpi.first.math.trajectory.TrapezoidProfile;import edu.wpi.first.math.util.Units;import edu.wpi.first.wpilibj.DriverStation;import edu.wpi.first.wpilibj.DriverStation.Alliance;import frc.lib.SwerveModuleConstants;public final class Constants{public static final class IntakeConstants{public static final int intakeMotorID=20;public static final int beamSensorPort=3;}public static final class ShooterConstants{public static final int beltMotorID=22;public static final int shooterMotorID=21;public static final int beamSensorPort=0;public static final double noteSpeedMetersPerSecond=20;public static final double relativeShooterAngle=112;public static final double shooterWheelSpeed=3500;public static final double shootTimeSeconds=0;public static final double kP=0;public static final double kI=0;public static final double kD=0;public static final double kFF=12.0/4500.0;}public static final class ArmConstants{public static final int LmotorID=41;public static final int RmotorID=42;public static final int encoderPort=1;public static final double minArmPosition=19;public static final double maxArmPosition=99;public static final double handoffPosition=20;public static final double ampPosition=98;public static final double kP=0.04;public static final double kI=0;public static final double kD=0;public static final InterpolatingTreeMap<Double,Double> ArmAngleMap=new InterpolatingDoubleTreeMap();static{ArmAngleMap.put(1.23,27.5);ArmAngleMap.put(1.27,31.0);ArmAngleMap.put(2.03,41.5);ArmAngleMap.put(2.26,47.1);ArmAngleMap.put(3.00,49.5);}public static final double maxShootingDistance=2.7;}public static final class FieldConstants{public static final double FIELD_LENGTH=16.4846;public static final double FIELD_WIDTH=8.1026;public static boolean isRedAlliance(){return DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get()==Alliance.Red;}public static Translation2d flipTranslationForAlliance(Translation2d translation){return isRedAlliance()? new Translation2d(FIELD_LENGTH - translation.getX(),translation.getY()):translation;}public static Pose2d flipPoseForAlliance(Pose2d pose){return isRedAlliance()? new Pose2d(FIELD_LENGTH - pose.getX(),pose.getY(),Rotation2d.fromDegrees(180).minus(pose.getRotation())):pose;}public static final Pose2d BLUE_AMP_SCORING=new Pose2d(1.83,7.57,Rotation2d.fromDegrees(-90));public static final Pose2d RED_AMP_SCORING=new Pose2d(FIELD_LENGTH - BLUE_AMP_SCORING.getX(),BLUE_AMP_SCORING.getY(),Rotation2d.fromDegrees(180).minus(BLUE_AMP_SCORING.getRotation()));public static final Translation2d RED_SPEAKER_LOCATION=new Translation2d(FIELD_LENGTH,5.56);public static final Translation2d BLUE_SPEAKER_LOCATION=new Translation2d(0,5.56);}public static final class ClimberConstants{public static final int MotorIDLeft=32;public static final int MotorIDRight=31;}public static final class AutoConstants{public static final HolonomicPathFollowerConfig pathConfig=new HolonomicPathFollowerConfig(new PIDConstants(5.0,0.00001,0.0),new PIDConstants(5.0,0.0005,0.001),SwerveConstants.maxSpeed,SwerveConstants.driveBaseRadius,new ReplanningConfig());public static final double maxVelocityAutoAlign=3;public static final double maxAccelerationAutoAlign=3;public static final double maxAngularVelocityAutoAlign=0;public static final double maxAngularAccelerationAutoAlign=0;public static final double rotation_kP=0.2;public static final double rotation_kI=0;public static final double rotation_kD=0;public static final double translation_kP=1;public static final double translation_kI=0;public static final double translation_kD=0;public static final TrapezoidProfile.Constraints autoAlignXYConstraints=new TrapezoidProfile.Constraints(maxVelocityAutoAlign,maxAccelerationAutoAlign);public static final TrapezoidProfile.Constraints autoAlignRConstraints=new TrapezoidProfile.Constraints(maxAngularVelocityAutoAlign,maxAngularAccelerationAutoAlign);public static final double ShootingAngle=35;}public static final class SwerveConstants{public static final double inputDeadband=.1;public static final int PIGEON_ID=17;public static final boolean invertPigeon=true;public static final double halfTrackWidth=Units.inchesToMeters(21.0/2.0);public static final double halfWheelBase=Units.inchesToMeters(21.0 /2.0);public static final double driveBaseRadius= Math.hypot(halfTrackWidth,halfWheelBase);public static final double wheelDiameter=(.0992);public static final double wheelCircumference=wheelDiameter * Math.PI;public static final double driveGearRatio=(8.14/1.0);public static final double angleGearRatio=(12.8/1.0);public static final double driveConversionPositionFactor=(wheelCircumference)/driveGearRatio;public static final double driveConversionVelocityFactor=driveConversionPositionFactor/60.0;public static final double angleConversionFactor=360.0/angleGearRatio;public static final double maxSpeed=5;public static final double maxAngularVelocity=maxSpeed/driveBaseRadius;public static final SwerveDriveKinematics swerveKinematics=new SwerveDriveKinematics(new Translation2d(halfWheelBase,halfTrackWidth),new Translation2d(-halfWheelBase,halfTrackWidth),new Translation2d(-halfWheelBase,-halfTrackWidth),new Translation2d(halfWheelBase,-halfTrackWidth));public static final double voltageComp=12.0;public static final int angleContinuousCurrentLimit=20;public static final int driveContinuousCurrentLimit=50;public static final double driveKP=0.1;public static final double driveKI=0.0;public static final double driveKD=0.0;public static final double driveKFF=0.0;public static final double driveKS=0.667;public static final double driveKV=2.44;public static final double driveKA=0.5;public static final double angleKP=0.01;public static final double angleKI=0.0;public static final double angleKD=0.0;public static final double angleKFF=0.0;public static final IdleMode angleNeutralMode=IdleMode.kBrake;public static final IdleMode driveNeutralMode=IdleMode.kBrake;public static final double openLoopRamp=0.25;public static final double closedLoopRamp=0.0;public static final boolean driveInvert=false;public static final boolean angleInvert=false;public static final boolean canCoderInvert=false;public static final boolean angleMotorInvert=false;public static final boolean driveMotorInvert=false;public static final class Mod0{public static final int driveMotorID=3;public static final int angleMotorID=2;public static final int canCoderID=11;public static final Rotation2d angleOffset=Rotation2d.fromDegrees(160.2);public static final SwerveModuleConstants constants= new SwerveModuleConstants(driveMotorID,angleMotorID,canCoderID,angleOffset);}public static final class Mod1{public static final int driveMotorID=5;public static final int angleMotorID=4;public static final int canCoderID=12;public static final Rotation2d angleOffset=Rotation2d.fromDegrees(117.2);public static final SwerveModuleConstants constants= new SwerveModuleConstants(driveMotorID,angleMotorID,canCoderID,angleOffset);}public static final class Mod2{public static final int driveMotorID=7;public static final int angleMotorID=6;public static final int canCoderID=13;public static final Rotation2d angleOffset=Rotation2d.fromDegrees(141);public static final SwerveModuleConstants constants= new SwerveModuleConstants(driveMotorID,angleMotorID,canCoderID,angleOffset);}public static final class Mod3{public static final int driveMotorID=9;public static final int angleMotorID=8;public static final int canCoderID=14 ;public static final Rotation2d angleOffset=Rotation2d.fromDegrees(-138);public static final SwerveModuleConstants constants= new SwerveModuleConstants(driveMotorID,angleMotorID,canCoderID,angleOffset);}}}