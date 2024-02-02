// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;


public class SwerveSubsystem extends SubsystemBase {
  private final Pigeon2 pigeon;
  

  
  private SwerveDrivePoseEstimator odometry;
  private SwerveModule[] swerveModules;

  private Vision vision = new Vision();
  private Field2d field;
  private CANdle leds = new CANdle(18);

  private double targetDirection = 0;
  public static enum LedMode{
    TELEOP,
    TEST,
  }
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
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

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeed,
      this::closedLoopDrive,
      SwerveConstants.pathConfig,
      () -> false,
      this
    );
  }


  //Simple field oriented drive. speeds range from -1 (full backwards) to 1 (full forwards)
  public void drive(double xSpeed, double ySpeed, double spinSpeed){
    drive(new Translation2d(xSpeed,ySpeed).times(SwerveConstants.maxSpeed), spinSpeed * SwerveConstants.maxAngularVelocity, true);
  }
  //drive the robot. translation: how fast you want to move in x and y direcitons, rotation: how fast you want to spin
  //fieldRelative: whether or not the controls are field or robot oriented, 
  public void drive(Translation2d translation, double rotation, boolean isFieldRelative)
  {
    //convert the inputs to a ChassisSpeeds
    driveFromChassisSpeeds(
          isFieldRelative
              //if you want field oriented driving, convert the desired speeds to robot oriented
              ? ChassisSpeeds.fromFieldRelativeSpeeds( translation.getX(), translation.getY(), rotation, getYaw())
              //if you want robot oriented driving, just generate the ChassisSpeeds
              
              :new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
              , true);
    
  }
  public void closedLoopDrive(ChassisSpeeds speeds){
    driveFromChassisSpeeds(speeds, false);
  }
  //drive the robot at a desired ChassisSpeeds. this is the core drive function that calculates the speed and direction of each module
  public void driveFromChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop){
    //get the target speed and direction for each module
    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    //if any module is asked to drive at more than 100% speed, then scale ALL 4 speeds until the max speed of any module is 100%
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

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
  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule module : swerveModules){
        positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }
  //get the speed and direction of each module as a list of SwerveModuleStates
  public SwerveModuleState[] getStates() {
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
    return (Constants.SwerveConstants.invertPigeon)
      ? 360 - yaw
      : yaw;
  }
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(getYawAsDouble());
  }

  

  
  public void spinInPlace(double speed){
    drive(0,0,speed);
  }
  public void spinToTarget(){
    spinInPlace(Math.min((targetDirection-getYawAsDouble())/50, 0.8));
    
  }
  public RepeatCommand alignToTarget(){
    return(new RepeatCommand(runOnce(() -> spinToTarget())));
  }
  public void snapToTargetPose(Pose2d targetPose){
    
  }
  public void setLeds(LedMode mode){
    switch (mode){
      case TELEOP:
        double tx = Math.abs(vision.getTx());
        if(vision.canSeeTarget()){
          if(tx < 2){
            leds.setLEDs(0,255,0);
          }
          else{
            
            leds.setLEDs(0,128,128);
          }
          
        }
        else{
          leds.setLEDs(255,0,0);
        }
      case TEST:
        leds.setLEDs(0,128,128);
    }
  }
  



  @Override
  public void periodic() {
    
    odometry.update(getYaw(), getPositions());
    if(vision.canSeeTarget()){
      targetDirection = getYaw().getDegrees() - vision.getTx();
    }
    
    if(vision.canSeeTarget() && vision.getTargetID() == 5){
      odometry.addVisionMeasurement(vision.getBotPose(),vision.getLatency());
    }
    //display estimated position on the driver station
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Pigeon Direction",  getYawAsDouble());
    
    SmartDashboard.putNumber("Target Direction",  targetDirection);
    
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Cancoder", module.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Integrated", module.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
    setLeds(LedMode.TELEOP);

    
  }
}

}
