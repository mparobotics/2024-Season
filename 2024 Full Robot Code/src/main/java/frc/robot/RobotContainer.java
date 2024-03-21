// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.auto.AutoModeSelector;
import frc.robot.auto.TestAuto;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AmpScore;
import frc.robot.commands.AngleAndShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeOveride;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController helmsController = new CommandXboxController(1);
  private final CommandJoystick buttonBox = new CommandJoystick(2);
  //private final CommandJoystick m_JoystickL = new CommandJoystick(0);
  //private final CommandJoystick m_JoystickR = new CommandJoystick(1);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric =
  new Trigger(driveController.leftBumper());


  

  /* Subsystems */
  private final SwerveSubsystem m_drive = new SwerveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimberSubsytem m_climber = new ClimberSubsytem();
  private final LEDController m_leds = new LEDController();


  private final AutoModeSelector m_autoModeSelector = new AutoModeSelector(m_arm,m_shooter,m_intake,m_drive);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    SmartDashboard.putData("Run Intake Until Full", new Intake(m_intake, m_arm, m_shooter));
    SmartDashboard.putData("Shoot", new Shoot(m_shooter, () -> true));
    
    
    m_shooter.setDefaultCommand(new InstantCommand(() -> m_shooter.setShooterSpeed(helmsController.getLeftTriggerAxis()), m_shooter));

    m_climber.setDefaultCommand(m_climber.climb(
          () -> buttonBox.getHID().getRawButton(4), 
          () -> buttonBox.getHID().getRawButton(5), 
          () -> buttonBox.getHID().getRawButton(9), 
          () -> buttonBox.getHID().getRawButton(10)));
    m_drive.setDefaultCommand(
      new TeleopSwerve(
          m_drive,
          () -> -getSpeedMultiplier() * driveController.getRawAxis(translationAxis),
          () -> -getSpeedMultiplier() * driveController.getRawAxis(strafeAxis),
          () -> -driveController.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean(),
          () -> driveController.getRightTriggerAxis() > 0.1 
          ));

    // Configure the trigger bindings
    configureBindings();
    m_drive.resetOdometry(new Pose2d());
    m_autoModeSelector.showOptions();
  }

  
  private void configureBindings() {   //makes the Y button on the controller zero the Gyro
    driveController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_drive.zeroGyro(), m_drive));

    //driveController.button(Button.kRightBumper.value).whileTrue(new MoveToPose(m_drive, () -> FieldConstants.isRedAlliance()? FieldConstants.RED_AMP_SCORING: FieldConstants.BLUE_AMP_SCORING));
    
    //helms right bumper button sets the arm in amp position
    helmsController.button(Button.kRightBumper.value).whileTrue(new AmpScore(m_arm, m_shooter, () -> helmsController.getLeftTriggerAxis() > 0.1));
    //left joystick up sets the arm position up 5 degs.
    helmsController.axisGreaterThan(Axis.kLeftY.value, 0.5).whileTrue(m_arm.setArmSetpointCommand(() -> m_arm.getArmPosition() - 2).repeatedly());
     //left joystick down moves the arm down 5 degs.
    helmsController.axisLessThan(Axis.kLeftY.value, -0.5).whileTrue(m_arm.setArmSetpointCommand(() -> m_arm.getArmPosition() + 2).repeatedly());

     //Makes the right trigger on the helms controller auto aim the arm 
    helmsController.axisGreaterThan(Axis.kRightTrigger.value, 0.1).whileTrue(new AimAndShoot(m_arm, m_shooter, () -> m_drive.getRelativeSpeakerLocation().getNorm(), () -> helmsController.getLeftTriggerAxis() > 0.1));

    helmsController.button(Button.kX.value).whileTrue(new AngleAndShoot(m_arm, m_shooter, () -> 25, () -> helmsController.getLeftTriggerAxis() > 0.1));
     //makes the right joystick run the intake until a note is intaked
    helmsController.axisGreaterThan(Axis.kRightY.value,0.5).whileTrue(new Intake(m_intake,m_arm,m_shooter));
    // makes the helms right joystick run the intake backwards when the joystick is moved backwards
    helmsController.axisLessThan(Axis.kRightY.value,-0.5).whileTrue(new ReverseIntake(m_intake, m_shooter));
    //The A button on the helms controller overides the intake
    helmsController.button(Button.kA.value).whileTrue(new IntakeOveride(m_intake, m_arm, m_shooter));
  }
  private double getSpeedMultiplier(){
    return driveController.getLeftTriggerAxis() > 0.1? 1: 0.7;
  }
  public void disabledPeriodic(){
    m_leds.disabledPeriodic();
  }
  public void autoPeriodic(){
    m_leds.autoPeriodic(m_shooter.isNoteInShooter());
  }
  public void teleopPeriodic(){
    m_leds.teleopPeriodic(m_shooter.isNoteInShooter(), m_shooter.isAtShootingSpeed(), m_arm.getArmPosition() > 23);
  }
  public void teleopInit(){
    m_arm.setToHandoffAngle();
  }
  public Command getAutonomousCommand() {
    return m_autoModeSelector.getSelectedAuto();
    
  }
}
