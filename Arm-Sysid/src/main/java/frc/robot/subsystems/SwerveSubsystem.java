// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SysIdSwerveSteerGains;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.SwerveConstants;


public class SwerveSubsystem extends SubsystemBase {

  private final SysIdRoutine.Config config = new SysIdRoutine.Config();

  //define measurement variables for the voltage going to the motors, the arm's angle, and the arm's angular velocoity.
  private final MutableMeasure<Voltage> drive_motor_voltage = MutableMeasure.ofBaseUnits(0,Units.Volts);

  private final MutableMeasure<Distance> drive_distance = MutableMeasure.ofBaseUnits(0,Units.Meters);

  private final MutableMeasure<Velocity<Distance>> arm_velocity = MutableMeasure.ofBaseUnits(0, Units.MetersPerSecond);

  private final Mechanism swerve_mechanism = new Mechanism(
    (Measure<Voltage> volts) -> {driveFromVoltage(volts.in(Units.Volts));}, //code that runs the mechanism goes here. must use a Measure<Voltage> to supply voltage to the motors, 
                (SysIdRoutineLog log) -> logDriveState(log), 
                this);

  private final SysIdRoutine swerve_sysid = new SysIdRoutine(config, swerve_mechanism);

  private final Pigeon2 pigeon;
  

  
  private SwerveDrivePoseEstimator odometry;
  private SwerveModule[] swerveModules;

  
  private Field2d field;
  
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    SmartDashboard.putData("Swerve: Run Quasistatic Forward",swerve_sysid.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Swerve: Run Quasistatic Reverse",swerve_sysid.quasistatic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Swerve: Run Dynamic Forward",swerve_sysid.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Swerve: Run Dynamic Reverse",swerve_sysid.dynamic(SysIdRoutine.Direction.kReverse));

    //instantiates new pigeon gyro, wipes it, and zeros it
    pigeon = new Pigeon2(SwerveConstants.PIGEON_ID);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    //list of all four swerve modules
    swerveModules =
    new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.Mod0.constants),
      new SwerveModule(1, SwerveConstants.Mod1.constants),
      new SwerveModule(2, SwerveConstants.Mod2.constants),
      new SwerveModule(3, SwerveConstants.Mod3.constants)
    };

    
    odometry = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getYaw(), getPositions(),new Pose2d(0,0,Rotation2d.fromDegrees(0)));
    
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    

    
  }

  public void driveFromVoltage(double volts){
    for(SwerveModule module: swerveModules){
      module.driveVolts(volts);
      
    }
    SmartDashboard.putNumber("Input Voltage", volts);
    SmartDashboard.putNumber("Distance", getPose().getX());
    SmartDashboard.putNumber("Velocity", getRobotRelativeSpeed().vxMetersPerSecond);

  }
  public void driveFromPercent(double percent){
    for(SwerveModule module: swerveModules){
      module.drivePercent(percent);
      
    }
    SmartDashboard.putNumber("Input Voltage", swerveModules[0].getVoltage());
    SmartDashboard.putNumber("Distance", getPose().getX());
    SmartDashboard.putNumber("Velocity", getRobotRelativeSpeed().vxMetersPerSecond);
  }
  public double getVoltage(){
    return swerveModules[0].getVoltage();
  }
  private void logDriveState(SysIdRoutineLog log){
    log.motor("Swerve Motors")
    .voltage(drive_motor_voltage.mut_replace(Units.Volts.of(getVoltage())))
    .linearPosition(drive_distance.mut_replace(Units.Meters.of(getPose().getX())))
    .linearVelocity(arm_velocity.mut_replace(Units.MetersPerSecond.of(getRobotRelativeSpeed().vxMetersPerSecond)));
  }
  //drive the robot. translation: how fast you want to move in x and y direcitons, rotation: how fast you want to spin
  //fieldRelative: whether or not the controls are field or robot oriented, 
  public void drive(double x, double y, double rotation, boolean isFieldRelative)
  {
    //convert the inputs to a ChassisSpeeds
    driveFromChassisSpeeds(
          isFieldRelative
              //if you want field oriented driving, convert the desired speeds to robot oriented
              ? ChassisSpeeds.fromFieldRelativeSpeeds( x, y, rotation, getYaw())
              //if you want robot oriented driving, just generate the ChassisSpeeds
              
              :new ChassisSpeeds(x, y, rotation)
              , true);
    
  }
  //drive with closed-loop control. used by pathplanner lib when following trajectories
  public void closedLoopDrive(ChassisSpeeds speeds){
    driveFromChassisSpeeds(speeds, false);
  }
  //drive the robot at a desired ChassisSpeeds. this is the core drive function that calculates the speed and direction of each module
  public void driveFromChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop){
    //get the target speed and direction for each module
    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    //if any module is asked to drive at more than 100% speed, then scale ALL 4 speeds until the max speed of any module is 100%
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    //set desired speed and direction for all 4 modules
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  

  
  //reset the pose to a given pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getPositions(), pose);
  }
  //set the current heading to be zero degrees
  public void zeroGyro() {
    pigeon.setYaw(0);
  }
  
  //get the robot's estimated location (in meters)
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }
  
  //get the distance traveled and direction of each module as a list of SwerveModulePositions
  private SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule module : swerveModules){
        positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }
  
  //get the speed and direction of each module as a list of SwerveModuleStates
  private SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }
  //get the speed of the robot in m/s and angular velocity in rad/s relative to the robot. Returns a ChassisSpeeds
  public ChassisSpeeds getRobotRelativeSpeed(){
    SwerveModuleState[] states = getStates();
    return SwerveConstants.swerveKinematics.toChassisSpeeds(states[0],states[1],states[2],states[3]);
  }
  //get the direction of the robot relative to its "zero" angle. this is usually pointing away from the driver station
  public double getYawAsDouble(){
    double yaw = pigeon.getAngle();
    return (SwerveConstants.invertPigeon)
      ? 360 - yaw
      : yaw;
  }
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(getYawAsDouble());
  }

  




  
 
 
  



  @Override
  public void periodic() {
    
    odometry.update(getYaw(), getPositions());
    //display estimated position on the driver station
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Pigeon Direction",  getYawAsDouble());
    
     
    
    
}

}
