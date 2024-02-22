// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDController;


public class Intake extends Command {
  private IntakeSubsystem m_intake;
  private LEDController m_led;


  /** Creates a new Intake. */
  public Intake(IntakeSubsystem intake, LEDController led) {
    addRequirements(intake);
    m_intake = intake;
    m_led = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (FieldConstants.isRedAlliance()){
      m_led.setAll(255, 0, 0);
    } else {
      m_led.setAll(0, 0, 255);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runIntake(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runIntake(0);
    if (m_intake.isNoteInIntake()) {
      m_led.setAll(255, 115, 0);
    } else {
      m_led.setAll(0, 0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.isNoteInIntake();
  }
}
