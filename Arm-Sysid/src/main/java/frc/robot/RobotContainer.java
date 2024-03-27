// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveVoltage;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
  
 
  private void configureBindings() {
    //m_armSubsystem.setDefaultCommand(m_armSubsystem.controlArmWithJoystick(() -> xbox.getLeftY()));
   // m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.controlIntakeWithJoystick(() -> xbox.getRightY()));
    //m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.controlShooterWithJoystick(() -> xbox.getLeftTriggerAxis(), () -> xbox.getRightTriggerAxis()));
    m_SwerveSubsystem.setDefaultCommand(new SwerveDriveVoltage(m_SwerveSubsystem,() ->  xbox.getLeftY()));
    xbox.button(Button.kA.value).onTrue(m_armSubsystem.stopMotors());
  }

  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
