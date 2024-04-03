// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SweepAutoIntakeControl extends Command {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  int notes_intaked = 0;
  boolean has_note = true;
  /** Creates a new SweepAutoIntakeControl. */
  public SweepAutoIntakeControl(IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
    m_intake = intake;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    notes_intaked = 0;
    has_note = m_shooter.isNoteInShooter();
    m_intake.runIntake(1);
    m_shooter.setBeltSpeed(0.75);
    m_shooter.setShooterSpeed(0.35);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //if we have a note now and didn't before, then we just intook a note! -> increase the intake count
    if(m_shooter.isNoteInShooter() && !has_note){
      notes_intaked++;
    }
    has_note = m_shooter.isNoteInShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setBeltSpeed(0);
    m_intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return notes_intaked == 5;
  }
}
