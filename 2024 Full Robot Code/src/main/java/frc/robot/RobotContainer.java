// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoModeSelector;
import frc.robot.auto.OneCenterNote;
import frc.robot.auto.AutoModeSelector.AutoMode;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
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


  private final AutoModeSelector m_autoModeSelector = new AutoModeSelector(m_arm,m_shooter,m_intake,m_drive,m_leds);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    SmartDashboard.putData("Run Intake Until Full", new Intake(m_intake, m_arm, m_shooter, m_leds));
    m_arm.setDefaultCommand(m_arm.teleopArmControlCommand(() -> helmsController.getLeftY()));
    m_shooter.setDefaultCommand(m_shooter.shooterControlCommand(() -> helmsController.getLeftTriggerAxis() > 0.5? 1:0, () -> helmsController.getRightY()));
    m_intake.setDefaultCommand(m_intake.IntakeControlCommand(() -> helmsController.getRightY()));
    m_leds.setDefaultCommand(m_leds.idleLedPattern());
    m_climber.setDefaultCommand(m_climber.climb(() -> buttonBox.getHID().getRawButton(4), () -> buttonBox.getHID().getRawButton(2), () -> buttonBox.getHID().getRawButton(3), () -> buttonBox.getHID().getRawButton(1)));
    m_drive.setDefaultCommand(
      new TeleopSwerve(
          m_drive,
          () -> -driveController.getRawAxis(translationAxis),
          () -> -driveController.getRawAxis(strafeAxis),
          () -> -driveController.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean(),
          () -> false 
          ));

    // Configure the trigger bindings
    configureBindings();
    m_drive.resetOdometry(new Pose2d());
    m_autoModeSelector.showOptions();
  }
  private double climberInput(boolean up, boolean down){
    return up? 1: (down? -1: 0);
  }
  
  private void configureBindings() {
    driveController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_drive.zeroGyro()));
    helmsController.button(Button.kRightBumper.value).whileTrue(new InstantCommand(() -> {m_shooter.setBeltSpeed(-0.3); m_intake.runIntake(-0.6);}));
    
    
  }
  public Command getAutonomousCommand() {
    //return new OneCenterNote(m_drive, m_intake, m_shooter, m_arm, m_leds);
    return m_autoModeSelector.getSelectedAuto();

  }
}
