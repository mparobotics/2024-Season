// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;




public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new CommandXboxController(0);
  //private final CommandJoystick m_JoystickL = new CommandJoystick(0);
  //private final CommandJoystick m_JoystickR = new CommandJoystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric =
  new Trigger(driveController.leftBumper());


  

  /* Subsystems */
  private final SwerveSubsystem m_Drive = new SwerveSubsystem();
  private final ArmSubsystem m_Arm = new ArmSubsystem();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem();


  final LEDController m_leds = new LEDController();


  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Drive.setDefaultCommand(
      new TeleopSwerve(
          m_Drive,
          () -> -driveController.getRawAxis(translationAxis),
          () -> -driveController.getRawAxis(strafeAxis),
          () -> -driveController.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean(),
          () -> false,
          () -> false
          
          ));

    // Configure the trigger bindings
    configureBindings();
  }

  
  private void configureBindings() {
    driveController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_Drive.zeroGyro()));
    driveController.button(Button.kRightBumper.value).whileTrue(m_Drive.alignToTarget());
  }

  Pose2d waypoint(double x, double y, double r){
    return new Pose2d(x,y, Rotation2d.fromDegrees(r));
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      waypoint(0,0,0),
      waypoint(3,0,0)
      
    );
    PathPlannerPath testPath = new PathPlannerPath(
      bezierPoints,
      new PathConstraints(4.0,3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
      new GoalEndState(0.0, Rotation2d.fromDegrees(180)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    PathPlannerPath drawnPath = PathPlannerPath.fromPathFile("testpath01");
    AutoBuilder.followPath(testPath);
    return new SequentialCommandGroup(new InstantCommand(() -> m_Drive.resetOdometry(waypoint(2, 2, 0))) , AutoBuilder.followPath(drawnPath));
  }
}
