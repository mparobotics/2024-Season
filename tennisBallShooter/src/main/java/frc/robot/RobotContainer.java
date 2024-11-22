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
  //m: naming notation, not original code, it's a copy.
  private final CommandXboxController controller = new CommandXboxController(1);
  //We are using a Xbox controller.
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() { //void: don't have set value, not returning any value.
    controller.axisGreaterThan(Axis.kRightY.value,0.5).whileTrue(m_MotorSubsystem.RunMotors().repeatedly());
    //y-axis, pass 0.5 the motor will spin up.
    controller.axisLessThan(Axis.kRightY.value,-0.5).whileTrue(m_MotorSubsystem.InverseMotors().repeatedly());
    //y-axis, pass -0.5 the motor will spin up the other way.
    m_MotorSubsystem.setDefaultCommand(m_MotorSubsystem.StopMotors());
    //Defalut for the motor, to stop the motor.
    //.repeatedly() makes command keep running repeatedly as long as joystick is pressed down
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
