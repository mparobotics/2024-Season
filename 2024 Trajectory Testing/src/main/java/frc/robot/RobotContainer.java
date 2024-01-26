// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
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
  private final CommandXboxController m_XboxController = new CommandXboxController(0);
  //private final CommandJoystick m_JoystickL = new CommandJoystick(0);
  //private final CommandJoystick m_JoystickR = new CommandJoystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric =
  new Trigger(m_XboxController.leftBumper());


  /*private final int translationAxis = Joystick.AxisType.kY.value; //left flight stick
  private final int strafeAxis = Joystick.AxisType.kX.value; //left flight stick
  private final int rotationAxis = Joystick.AxisType.kX.value; //right flight stick*/

  /* Subsystems */
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();


  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_SwerveSubsystem.setDefaultCommand(
      new TeleopSwerve(
          m_SwerveSubsystem,
          () -> -m_XboxController.getRawAxis(translationAxis),
          () -> -m_XboxController.getRawAxis(strafeAxis),
          () -> -m_XboxController.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()));

    // Configure the trigger bindings
    configureBindings();
  }

  
  private void configureBindings() {
    m_XboxController.button(Button.kA.value).onTrue(m_ArmSubsystem.upPosition());
    m_XboxController.button(Button.kB.value).onTrue(m_ArmSubsystem.downPosition());
    m_XboxController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    m_XboxController.button(Button.kRightBumper.value).whileTrue(m_SwerveSubsystem.alignToTarget());
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
    return new SequentialCommandGroup(new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(waypoint(0, 0, 0))) ,AutoBuilder.followPath(drawnPath));
  }
}
