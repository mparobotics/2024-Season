// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexNote extends Command {
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  /** A command that lowers the arm and feeds a note into the indexed position. */
  public IndexNote(ArmSubsystem arm, ShooterSubsystem shooter, IntakeSubsystem intake) {
    
    addRequirements(arm);
    addRequirements(shooter);
    addRequirements(intake);
    
    m_arm = arm;
    m_shooter = shooter;
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setTarget(ArmConstants.handoffPosition);
    m_shooter.setBeltSpeed(1);
    m_intake.runIntake(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.isAtTarget()){
      m_intake.runIntake(1);
    }
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
    return m_arm.isAtTarget() && m_shooter.isNoteInShooter();
  }
}
