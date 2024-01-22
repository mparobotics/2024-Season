// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final WPI_Pigeon2 pigeon;
  

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;

  private Field2d field;
  private CANdle leds = new CANdle(18);

  private double targetDirection = 0;
  public static enum LedMode{
    TELEOP,
    TEST,
  }
  NetworkTable getLimelight(){
    return NetworkTableInstance.getDefault().getTable("limelight");
  }
  boolean canSeeTarget(){
    return getLimelight().getEntry("tv").getDouble(0) == 1;
  }
  double getTx(){
    return getLimelight().getEntry("tx").getDouble(0);
  }
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //instantiates new pigeon gyro, wipes it, and zeros it
    pigeon = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_ID);
    pigeon.configFactoryDefault();
    zeroGyro();

    //list of all four swerve modules
    swerveModules =
    new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
      new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
      new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
      new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    //odometery will track where the robot is on the field
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

    //display a (outdated) field on SmartDashboard
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeed,
      this::driveFromChassisSpeeds,
      SwerveConstants.pathConfig,
      () -> false,
      this
    );
  }


  //Simple field oriented drive speeds range from -1 (full backwards) to 1 (full forwards)
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
              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    
  }
  //drive the robot at a desired ChassisSpeeds. this is the core drive function that calculates the speed and direction of each module
  public void driveFromChassisSpeeds(ChassisSpeeds speeds){
    //get the target speed and direction for each module
    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    //if any module is asked to drive at more than 100% speed, then scale ALL 4 speeds until the max speed of any module is 100%
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    //set desired speed and direction for all 4 modules
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(swerveModuleStates[module.moduleNumber], true);
    }
  }

  
  

  
  //reset the pose to a given pose
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }
  //set the current heading to be zero degrees
  public void zeroGyro() {
    pigeon.setYaw(0);
  }
  
  //get the robot's estimated location (in meters)
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
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
  public Rotation2d getYaw() {
    return (Constants.SwerveConstants.invertPigeon)
      ? Rotation2d.fromDegrees(360 - pigeon.getYaw())
      : Rotation2d.fromDegrees(pigeon.getYaw());
  }

  

  
  public void spinInPlace(double speed){
    drive(0,0,speed);
  }
  public void spinToTarget(){
    spinInPlace(Math.min((targetDirection-pigeon.getYaw())/50, 0.8));
    
  }
  public RepeatCommand alignToTarget(){
    return(new RepeatCommand(runOnce(() -> spinToTarget())));
  }
  public void setLeds(LedMode mode){
    switch (mode){
      case TELEOP:
        double tx = Math.abs(getTx());
        if(canSeeTarget()){
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
      }
    }
  



  @Override
  public void periodic() {
    //keep the odometry updated
    swerveOdometry.update(getYaw(), getPositions());
    if(canSeeTarget()){
      targetDirection = getYaw().getDegrees() - getTx();
    }
    //display estimated position on the driver station
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Pigeon Direction",  pigeon.getYaw());
    
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
