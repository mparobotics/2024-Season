// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;


public class SwerveSubsystem extends SubsystemBase {
  private final Pigeon2 pigeon;
  

  
  private SwerveDrivePoseEstimator odometry;
  private SwerveModule[] swerveModules;

  
  private Field2d field;
  
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //instantiates new pigeon gyro, wipes it, and zeros it
    pigeon = new Pigeon2(SwerveConstants.PIGEON_ID);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    configPathPlanner();

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

  public Translation2d getRelativeSpeakerLocation(){
    Translation2d targetLocation = FieldConstants.isRedAlliance()? FieldConstants.RED_SPEAKER_LOCATION: FieldConstants.BLUE_SPEAKER_LOCATION;
    return targetLocation.minus(getPose().getTranslation());
  }
 
  public Translation2d getVirtualTarget(){
    /* When trying to shoot while moving, we will often miss if we aim directly at the speaker. This is because the motion of the 
    robot affects the note's velocity leaving the shooter, which affects where it ends up going and how fast it moves. The goal of this function is to use
    the robot's velocity and some math to estimate the location of a "virtual target" that we should aim at in order to get our note
    to fly straight towards the speaker. 
   */
    //get the speaker location relative to the robot (blue side always coordinates)
    Translation2d speakerLocation = getRelativeSpeakerLocation();

    //get the speed of the robot as a chassisSpeeds
    ChassisSpeeds chassisSpeeds = getRobotRelativeSpeed();
    //make a translation2d with the x and y components of the velocity of the robot
    Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    //Calculate the component of the velocity parallel to the speaker direction, and the component going perpendicular to the speaker direction
    Translation2d rotatedVelocity = velocity.rotateBy(speakerLocation.getAngle().times(-1));
    //The robot's velocity is always moving the note off of the straight path to the speaker, so we want to pick a shooting direction that cancels out the perpendicular velocity
    double shootDirection = Math.asin(-rotatedVelocity.getY() / ShooterConstants.noteSpeedMetersPerSecond);
    
    //make a translation2d of the note's velocity as it leaves the shooter
    Translation2d relativeNoteVelocity = new Translation2d(ShooterConstants.noteSpeedMetersPerSecond, new Rotation2d(shootDirection));

    //The parallel components of the robot velocity and the shooter velocity are added together together to find how fast the note is approaching the speaker.
    //We calculate how long the note will take to arrive at the speaker by dividing the distance to the speaker by the speed of the note
    double timeToSpeaker = speakerLocation.getNorm()/(relativeNoteVelocity.getX() + rotatedVelocity.getX());

    //the "virtual target" we should aim as is offset from the actual speaker location by the robot velocity times the amount of time the note will be in the air for
    return speakerLocation.minus(velocity.times(timeToSpeaker));
  }
  
  public Pose2d getIntakePose(){
    Translation2d relativeNoteLocation = Vision.getRelativeNoteLocation().rotateBy(getYaw().times(-1));
    Rotation2d targetDirection = relativeNoteLocation.getAngle();
    Translation2d noteLocation = getPose().getTranslation().plus(relativeNoteLocation);
    return new Pose2d(noteLocation, targetDirection);
  }
  

  public void configPathPlanner(){
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeed,
      this::closedLoopDrive,
      AutoConstants.pathConfig,
      () -> (DriverStation.getAlliance().get() ==  Alliance.Red),
      this
    );  
  }

  public Command followPathFromFile(String filename){
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile(filename));
  }

  
 
 
  



  @Override
  public void periodic() {
    
    odometry.update(getYaw(), getPositions());
    
    
    if(Vision.canSeeAprilTag()){
      //odometry.addVisionMeasurement(Vision.getBotPose(),Vision.getLatency());
    }
    
    //display estimated position on the driver station
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Pigeon Direction",  getYawAsDouble());
    getVirtualTarget();
    
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Cancoder", module.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Integrated", module.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(
        "Mod" + module.moduleNumber + " Distance", module.getPosition().distanceMeters);
      

    
    }
    
}

}
