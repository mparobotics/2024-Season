// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;




public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  MotorSubsystem m_MotorSubsystem = new MotorSubsystem();


  private final CommandXboxController xbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  
  private void configureBindings() {
    //control the motor with the left joystick
    //m_MotorSubsystem.setDefaultCommand(m_MotorSubsystem.setPositionPID(() -> xbox.getLeftY()));
    m_MotorSubsystem.setDefaultCommand(m_MotorSubsystem.setMotor(() -> xbox.getLeftY()));
    xbox.button(Button.kA.value).whileTrue(m_MotorSubsystem.shoot());
    
  }


  public Command getAutonomousCommand() {
    //no auto (yet...)
    return null;
  }
}
