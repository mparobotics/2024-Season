// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
  private ShooterSubsystem m_shooter;
  private BooleanSupplier m_shouldShoot;

  private boolean shooterIsEmpty;
  private Timer m_timer = new Timer();
  /** A Command that shoots a note*/
  public Shoot(ShooterSubsystem shooter, BooleanSupplier shootingSupplier) {
    addRequirements(shooter);
  
    m_shooter = shooter;
    m_shouldShoot = shootingSupplier;
    m_timer.stop();
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.spinUpShooter();
    shooterIsEmpty = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shooter.isAtShootingSpeed() && m_shouldShoot.getAsBoolean()){
      m_shooter.setBeltSpeed(1);
    }
    if(!m_shooter.isNoteInShooter() && !shooterIsEmpty){
      shooterIsEmpty = true;
      m_timer.reset();
      m_timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_shooter.isNoteInShooter() && m_timer.get() > ShooterConstants.shootTimeSeconds;
    
  }
}
