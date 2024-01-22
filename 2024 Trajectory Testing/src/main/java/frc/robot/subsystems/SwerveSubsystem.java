// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
<<<<<<< Updated upstream
=======
import com.pathplanner.lib.auto.AutoBuilder;
>>>>>>> Stashed changes

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
<<<<<<< Updated upstream
=======

import edu.wpi.first.wpilibj2.command.RepeatCommand;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final WPI_Pigeon2 pigeon;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;

  private Field2d field;
<<<<<<< Updated upstream

=======
  
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
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
=======

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeed,
      this::driveFromChassisSpeeds,
      SwerveConstants.pathConfig,
      () -> false,
      this
    );
>>>>>>> Stashed changes
  }


  //Simple field oriented drive speeds range from -1 (full backwards) to 1 (full forwards)
  public void drive(double xSpeed, double ySpeed, double spinSpeed){
    drive(new Translation2d(xSpeed,ySpeed).times(SwerveConstants.maxSpeed), spinSpeed * SwerveConstants.maxAngularVelocity, true);
  }
  //drive the robot. translation: how fast you want to move in x and y direcitons, rotation: how fast you want to spin
  //fieldRelative: whether or not the controls are field or robot oriented, 
  public void drive(Translation2d translation, double rotation, boolean isFieldRelative)
  {
<<<<<<< Updated upstream
    SwerveModuleState[] swerveModuleStates =
      Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          //fancy way to do an if else statement 
          //if field relative == true, use field relative stuff, otherwise use robot centric
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  translation.getX(), translation.getY(), rotation, getYaw())
              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
  //sets to top speed if above top speed
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

  //set states for all 4 modules
  for (SwerveModule mod : mSwerveMods) {
    mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
  }
}

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

=======
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

  
  
>>>>>>> Stashed changes

  
  //reset the pose to a given pose
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }
<<<<<<< Updated upstream

  public void setWheelsToX() {
    setModuleStates(new SwerveModuleState[] {
      // front left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      // front right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      // back left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
      // back right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}


=======
  //set the current heading to be zero degrees
>>>>>>> Stashed changes
  public void zeroGyro() {
    pigeon.setYaw(0);
  }
  
  //get the robot's estimated location (in meters)
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }
<<<<<<< Updated upstream

  public boolean AutoBalance(){
    double roll_error = pigeon.getPitch();//the angle of the robot
    double balance_kp = -.005;//Variable muliplied by roll_error
    double position_adjust = 0.0;
    double min_command = 0.0;//adds a minimum input to the motors to overcome friction if the position adjust isn't enough
    if (roll_error > 6.0)
    {
      position_adjust = balance_kp * roll_error + min_command;//equation that figures out how fast it should go to adjust
      //position_adjust = Math.max(Math.min(position_adjust,.15), -.15);  this gets the same thing done in one line
      if (position_adjust > .1){position_adjust = .1;}
      if (position_adjust < -.1){position_adjust = -.1;}
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      
      return false;
=======
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
    drive(new Translation2d(0,0),speed * Constants.SwerveConstants.maxAngularVelocity,false);
  }
  public void spinToTarget(){
    spinInPlace((pigeon.getYaw()-180)/180);
    
  }
  public RepeatCommand alignToTarget(){
    return(new RepeatCommand(runOnce(() -> spinToTarget())));
  }
  public void setLeds(LedMode mode){
    switch (mode){
      case TELEOP:
        double tx = Math.abs(getTx());
        if(canSeeTarget()){
          if(tx < 10){
            leds.setLEDs(0,255,0);
          }
          else{
            
            leds.setLEDs(0,128,128);
          }
          
        }
        else{
          leds.setLEDs(255,0,0);
        }
>>>>>>> Stashed changes
    }
    else if (roll_error < -6.0)
    {
      position_adjust = balance_kp * roll_error - min_command;
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      if (position_adjust > .3){position_adjust = .3;}
      if (position_adjust < -.3){position_adjust = -.3;}
      return false;
    }
    else{
      drive(new Translation2d(0, 0), 0.0, true, false);
      return true;}
    
  }



  @Override
  public void periodic() {
<<<<<<< Updated upstream
        swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Pigeon Roll",  pigeon.getPitch());

    for (SwerveModule mod : mSwerveMods) {
=======
    //keep the odometry updated
    swerveOdometry.update(getYaw(), getPositions());

    //display estimated position on the driver station
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Pigeon Direction",  pigeon.getYaw());
    
    for (SwerveModule module : swerveModules) {
>>>>>>> Stashed changes
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Cancoder", module.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Integrated", module.getState().angle.getDegrees());
      SmartDashboard.putNumber(
<<<<<<< Updated upstream
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
=======
          "Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
    setLeds(LedMode.TELEOP);
>>>>>>> Stashed changes
  }
}

}
