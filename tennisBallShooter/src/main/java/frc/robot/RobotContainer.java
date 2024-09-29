// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  
  private final MotorSubsystem m_MotorSubsystem = new MotorSubsystem();
  private final CommandXboxController controller = new CommandXboxController(0);
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.axisGreaterThan(Axis.kRightY.value,0.5).whileTrue(m_MotorSubsystem.RunMotors());
    controller.axisLessThan(Axis.kRightY.value,-0.5).whileTrue(m_MotorSubsystem.RunMotors());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
