// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final WPI_Pigeon2 pigeon;

  private final CANdle leds = new CANdle(18);
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;
  
  public static enum LedMode{
    TELEOP,
    TEST,
  }
  public NetworkTable getLimelight(){
    return  NetworkTableInstance.getDefault().getTable("limelight");
  }
  public boolean canSeeTarget(){
    double tv = getLimelight().getEntry("tv").getDouble(0);
    return tv == 1.0;
  }
  public double getTx(){
    return getLimelight().getEntry("tx").getDouble(0);
  }
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //instantiates new pigeon gyro, wipes it, and zeros it
    pigeon = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_ID);
    pigeon.configFactoryDefault();
    zeroGyro();

    //Creates all four swerve modules into a swerve drive
    mSwerveMods =
    new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
      new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
      new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
      new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    //creates new swerve odometry (odometry is where the robot is on the field)
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

    //puts out the field
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeeds,
      this::driveFromChassisSpeeds,
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
      () -> false,
      this
    );
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  //takes the coordinate on field wants to go to, the rotation of it, whether or not in field relative mode, and if in open loop control
  {
    driveFromChassisSpeeds(
          //fancy way to do an if else statement 
          //if field relative == true, use field relative stuff, otherwise use robot centric
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  translation.getX(), translation.getY(), rotation, getYaw())
              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    
  }
  public void driveFromChassisSpeeds(ChassisSpeeds speeds){
    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    //sets to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    //set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
    }
  }
  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }
  

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

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
  public ChassisSpeeds getRobotRelativeSpeeds(){
    SwerveModuleState[] states = getStates();
    return SwerveConstants.swerveKinematics.toChassisSpeeds(states[0],states[1],states[2],states[3]);
  }
  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}


  public void zeroGyro() {
    pigeon.setYaw(0);
  }

  public Rotation2d getYaw() {
    //fancy if else loop again
    return (Constants.SwerveConstants.invertPigeon)
        ? Rotation2d.fromDegrees(360 - pigeon.getYaw())
        : Rotation2d.fromDegrees(pigeon.getYaw());
  }
  public void spinInPlace(double speed){
    drive(new Translation2d(0,0),speed * Constants.SwerveConstants.maxAngularVelocity,false,true);
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
    }
  }
  



  @Override
  public void periodic() {

        swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Pigeon Direction",  pigeon.getYaw());
    
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    setLeds(LedMode.TELEOP);
  }
}

}
