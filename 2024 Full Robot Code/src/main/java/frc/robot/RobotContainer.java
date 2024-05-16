// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.auto.AutoModeSelector;

import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AmpScore;
import frc.robot.commands.AngleAndShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeOverride;
import frc.robot.commands.MeasureWheelDiameter;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
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
  private final Joystick buttonHID = buttonBox.getHID();
  //private final CommandJoystick m_JoystickL = new CommandJoystick(0);
  //private final CommandJoystick m_JoystickR = new CommandJoystick(1);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric =
  new Trigger(driveController.leftBumper());


  private Trigger shoot = helmsController.axisGreaterThan(Axis.kLeftTrigger.value,0.1);

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
    SmartDashboard.putData("set to midfield", m_drive.resetOdometryCommand(FieldConstants.FIELD_LENGTH/ 2, FieldConstants.FIELD_WIDTH / 2));
    SmartDashboard.putData("set to subwoofer", m_drive.resetOdometryCommand(1.34, 5.54));
    SmartDashboard.putData("test wheel diameter", new MeasureWheelDiameter(m_drive));
    m_shooter.setDefaultCommand(m_shooter.defaultShooterCommand(
      shoot,
      () -> m_drive.isInSpinUpRange()
       ));

    m_climber.setDefaultCommand(m_climber.climb(
      () -> buttonHID.getRawButton(4), 
      () -> buttonHID.getRawButton(5), 
      () -> buttonHID.getRawButton(9), 
      () -> buttonHID.getRawButton(10)
    ));
    m_drive.setDefaultCommand(
      new TeleopSwerve(
          m_drive,
          () -> -getSpeedMultiplier() * driveController.getRawAxis(translationAxis),
          () -> -getSpeedMultiplier() * driveController.getRawAxis(strafeAxis),
          () -> -driveController.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean(),
          () -> driveController.getRightTriggerAxis() > 0.1,
          () -> driveController.getHID().getRawButton(Button.kX.value) 
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
    helmsController.button(Button.kRightBumper.value).whileTrue(new AmpScore(m_arm, m_shooter, shoot));
    //helms left joystick manually moves the arm up and down
    helmsController.axisGreaterThan(Axis.kLeftY.value, 0.5).whileTrue(m_arm.changeArmSetpointCommand(-2).repeatedly());
    helmsController.axisLessThan(Axis.kLeftY.value, -0.5).whileTrue(m_arm.changeArmSetpointCommand(2).repeatedly());

    //holding the right trigger on the helms controller auto aims the arm 
    helmsController.axisGreaterThan(Axis.kRightTrigger.value, 0.1).whileTrue(new AimAndShoot(m_arm, m_shooter, () -> m_drive.getRelativeSpeakerLocation().getNorm(), () -> helmsController.getLeftTriggerAxis() > 0.1));
    
    //X sets the arm to subwoofer angle
    helmsController.button(Button.kX.value).whileTrue(new AngleAndShoot(m_arm, m_shooter, () -> 25, shoot));
    //Y sets the arm to the podium angle
    helmsController.button(Button.kY.value).whileTrue(new AngleAndShoot(m_arm, m_shooter, () -> 47, shoot));
    buttonBox.button(3).whileTrue(new SlowShoot(m_arm,m_shooter, () -> 25, shoot)); 
  

    //B sets the arm to feeding angle
    helmsController.button(Button.kB.value).whileTrue(new AngleAndShoot(m_arm, m_shooter, () -> 69.3, shoot));

     //helms right joystick down runs the intake until a note is intaked
    helmsController.axisGreaterThan(Axis.kRightY.value,0.5).whileTrue(new Intake(m_intake,m_arm,m_shooter));
    // helms right joystick up runs the intake backwards
    helmsController.axisLessThan(Axis.kRightY.value,-0.5).whileTrue(new ReverseIntake(m_intake, m_shooter));
    //The A button on the helms controller runs the intake without using the beam break to check if its full
    helmsController.button(Button.kA.value).whileTrue(new IntakeOverride(m_intake, m_arm, m_shooter));
  }
  private double getSpeedMultiplier(){
 
    if (buttonHID.getRawButton(1)) {
      return 1;
      
    }
    return 0.25;
  
  }
  private double getTurnMultiplier(){
    if (buttonHID.getRawButton(1)){
      return 1;
    }
    return 0.25;
  }
  public void disabledPeriodic(){
    m_leds.disabledPeriodic();
  }
  public void autoPeriodic(){
    m_leds.autoPeriodic(m_shooter.isNoteInShooter());
  }
  public void teleopPeriodic(){
    m_leds.teleopPeriodic(m_shooter.isNoteInShooter(),m_drive.isInRange(), m_shooter.isAtShootingSpeed(), m_arm.getArmPosition() > 23);
  }
  public void teleopInit(){
    m_shooter.stopShooting(); 
    //11'8 prevention measure
    //puts the arm down at the start of teleop so that we don't smack the stage again
    m_arm.setToHandoffAngle();
  }
  public Command getAutonomousCommand() {
    return m_autoModeSelector.getSelectedAuto();
    
  }
}
