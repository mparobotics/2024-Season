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
import frc.robot.auto.AutoModeSelector.AutoMode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
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


  private final LEDController m_leds = new LEDController();


  private final AutoModeSelector m_autoModeSelector = new AutoModeSelector(m_arm,m_shooter,m_intake,m_drive,m_leds);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    m_arm.setDefaultCommand(m_arm.teleopArmControlCommand(() -> helmsController.getLeftY()));
    m_shooter.setDefaultCommand(m_shooter.shooterControlCommand(() -> helmsController.getLeftTriggerAxis() > 0.5? 1:0, () -> -helmsController.getRightY()));
    m_intake.setDefaultCommand(m_intake.IntakeControlCommand(() -> helmsController.getRightY()));
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
  }

  
  private void configureBindings() {
    driveController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_drive.zeroGyro()));
    helmsController.button(Button.kRightBumper.value).whileTrue(new InstantCommand(() -> {m_shooter.setBeltSpeed(-0.3); m_intake.runIntake(-0.6);}));
    

  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return m_autoModeSelector.getAuto(AutoMode.TEST);
    return new SequentialCommandGroup(
    new InstantCommand(() -> m_drive.resetOdometry(new Pose2d(16.02,2.04,Rotation2d.fromDegrees(180)))),
    m_intake.IntakeControlCommand(() -> 0.5),
    m_shooter.shooterControlCommand(() -> 0, () -> -0.5),
    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Kiwi")));
  }
}
