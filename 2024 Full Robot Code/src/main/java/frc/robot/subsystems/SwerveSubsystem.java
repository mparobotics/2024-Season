// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpersOld;
import frc.lib.LimelightHelpersOld.PoseEstimate;
import frc.robot.Constants.ArmConstants;
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
    SmartDashboard.putNumber("distance",0); 
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
    if(FieldConstants.isRedAlliance()){
      pigeon.setYaw(180);
    }
    else{
      pigeon.setYaw(0);
    }
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
  public boolean isInRange(){
    return getRelativeSpeakerLocation().getNorm() < ArmConstants.maxShootingDistance;
  }
  //Range to start spinning up the shooter. A little further out than our max shooting distance
  public boolean isInSpinUpRange(){
    return getRelativeSpeakerLocation().getNorm() < ArmConstants.maxShootingDistance + 2;
  }
  
 

 
  public Translation2d getVirtualTarget(){
    /* When trying to shoot while moving, we will often miss if we aim directly at the speaker. This is because the motion of the 
    robot affects the note's velocity leaving the shooter, which affects where it ends up going and how fast it moves. The goal of this function is to use
    the robot's velocity and some math to estimate the location of a "virtual target" that we should aim at in order to get our note
    to fly straight towards the speaker. 
   */
    //get the speaker location relative to the robot (blue side always coordinates)
    Translation2d speakerLocation = getRelativeSpeakerLocation();
    double estimatedShotAngle = Units.degreesToRadians(ArmConstants.ArmAngleMap.get(speakerLocation.getNorm()) + ShooterConstants.relativeShooterAngle);
    double noteSpeed = ShooterConstants.noteSpeedMetersPerSecond * Math.cos(estimatedShotAngle);
    //get the speed of the robot as a chassisSpeeds
    ChassisSpeeds chassisSpeeds = getRobotRelativeSpeed();
    //make a translation2d with the x and y components of the velocity of the robot
    Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    //Calculate the component of the velocity parallel to the speaker direction, and the component going perpendicular to the speaker direction
    Translation2d rotatedVelocity = velocity.rotateBy(speakerLocation.getAngle().times(-1));
    //The robot's velocity is always moving the note off of the straight path to the speaker, so we want to pick a shooting direction that cancels out the perpendicular velocity
    double shootDirection = Math.asin(-rotatedVelocity.getY() / noteSpeed);
    
    //make a translation2d of the note's velocity as it leaves the shooter
    Translation2d relativeNoteVelocity = new Translation2d(noteSpeed, new Rotation2d(shootDirection));

    //The parallel components of the robot velocity and the shooter velocity are added together together to find how fast the note is approaching the speaker.
    //We calculate how long the note will take to arrive at the speaker by dividing the distance to the speaker by the speed of the note
    double timeToSpeaker = speakerLocation.getNorm()/(relativeNoteVelocity.getX() + rotatedVelocity.getX());

    //the "virtual target" we should aim as is offset from the actual speaker location by the robot velocity times the amount of time the note will be in the air for
    return speakerLocation.minus(velocity.times(timeToSpeaker));
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
  public Command startAutoAt(double x,double y,double direction){
    return runOnce(() -> {
      Pose2d startPose = FieldConstants.flipPoseForAlliance(new Pose2d(x,y,Rotation2d.fromDegrees(direction)));
      pigeon.setYaw(startPose.getRotation().getDegrees());
      odometry.resetPosition(startPose.getRotation(), getPositions(), startPose);
    });
  }
  
  
  public void addVisionMeasurement(String limelightName){
    PoseEstimate visionPoseEstimate = LimelightHelpersOld.getBotPoseEstimate_wpiBlue(limelightName);
    double targetArea = LimelightHelpersOld.getTA(limelightName);
      SmartDashboard.putNumber(limelightName + " target area",targetArea);
      //ignore apriltags that are too small to get a reliable measurement, or pose estimates that place the robot outside of the field
      boolean isLimelightGood = targetArea > 0.15 && 
    
      visionPoseEstimate.pose.getTranslation().minus(new Translation2d(FieldConstants.FIELD_LENGTH / 2, FieldConstants.FIELD_WIDTH / 2)).getNorm() < 10;

      if(isLimelightGood){
        
        odometry.addVisionMeasurement(visionPoseEstimate.pose,visionPoseEstimate.timestampSeconds);
      }
      
  }
  public boolean isEstimateValid(LimelightHelpers.PoseEstimate estimate){
    boolean isInField = estimate.pose.getX() > 0 && estimate.pose.getX() < FieldConstants.FIELD_LENGTH
                      && estimate.pose.getY() > 0 && estimate.pose.getY() < FieldConstants.FIELD_WIDTH;
    boolean isCloseEnough = estimate.avgTagDist < 5;
    boolean canSeeTag = estimate.tagCount > 0;
    return  canSeeTag && isInField && isCloseEnough;
  }
  



  @Override
  public void periodic() {
    
    odometry.update(getYaw(), getPositions());
    LimelightHelpers.PoseEstimate estimateA = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-a");
    LimelightHelpers.PoseEstimate estimateB = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-b");

    boolean AisValid = isEstimateValid(estimateA);
    boolean BisValid = isEstimateValid(estimateB);

    SmartDashboard.putBoolean("limelight A is valid" , AisValid);
    SmartDashboard.putBoolean("limelight B is valid" , BisValid);
    

    //if each limelight can see one tag, then between the two of them we'll get a decent pose estimate
    if(estimateA.tagCount == 1 && estimateB.tagCount == 1 && AisValid && BisValid){
      //low standard deviations for translation, really high standard deviation for rotation since we want to just use the pigeon for heading
      odometry.setVisionMeasurementStdDevs(VecBuilder.fill(1,1,100000));

      odometry.addVisionMeasurement(estimateA.pose,estimateA.timestampSeconds);
      odometry.addVisionMeasurement(estimateB.pose,estimateB.timestampSeconds);
    }
    else{
      //If either limelight can see 2 tags simultaneously we can trust it enough to use its pose estimate
      //If just one limelight can see just one tag, then scale the standard deviations based on tag distance
      if(AisValid){
        if(estimateA.tagCount >= 2){
          odometry.setVisionMeasurementStdDevs(VecBuilder.fill(1,1,100000));
        }
        else{
          //scale standard deviations by the distance to the tag. (trust far away tags less)
          odometry.setVisionMeasurementStdDevs(VecBuilder.fill(2 / estimateA.avgTagDist,2 / estimateA.avgTagDist,100000));
        }
        odometry.addVisionMeasurement(estimateA.pose,estimateA.timestampSeconds);
      }
      if(BisValid){
        if(estimateB.tagCount >= 2){
          odometry.setVisionMeasurementStdDevs(VecBuilder.fill(1,1,100000));
        }
        else{
          //scale standard deviations by the distance to the tag. (trust far away tags less)
          odometry.setVisionMeasurementStdDevs(VecBuilder.fill(2 / estimateB.avgTagDist,2 / estimateB.avgTagDist,100000));
        }
        odometry.addVisionMeasurement(estimateB.pose,estimateB.timestampSeconds);
      }
    }
    
    
    /* 
    if(LimelightHelpersOld.getTV("limelight-a") && LimelightHelpersOld.getTV("limelight-b")){
      addVisionMeasurement("limelight-a");
      addVisionMeasurement("limelight-b");
    }
    */
    //display estimated position on the driver station
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Pigeon Direction",  getYawAsDouble());
    SmartDashboard.putNumber("position-X",getPose().getX()); 
    SmartDashboard.putNumber("position-Y",getPose().getY()); 

    
    
}

}
