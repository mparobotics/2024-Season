// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.subsystems.ShooterSubsystem;


public class AutoAmpScore extends Command {
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;


  
  /** Command to score a note in the AMP*/
  public AutoAmpScore(ArmSubsystem arm, ShooterSubsystem shooter) {
    addRequirements(arm, shooter);

    m_arm = arm;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setToAmpAngle();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setBeltSpeed(0.75);
    m_shooter.setShooterSpeed(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setToHandoffAngle();
    m_shooter.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_shooter.isNoteInShooter();
  }
}
