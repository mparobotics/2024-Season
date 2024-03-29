// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.subsystems.ShooterSubsystem;


public class AmpScore extends Command {
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  
  private BooleanSupplier m_shouldScore;

  
  /** Command to score a note in the AMP*/
  public AmpScore(ArmSubsystem arm, ShooterSubsystem shooter, BooleanSupplier shouldScore) {
    addRequirements(arm, shooter);

    m_arm = arm;
    
    m_shooter = shooter;
    m_shouldScore = shouldScore;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //move the arm up and spin the shooter at low speed
    m_arm.setToAmpAngle();
    m_shooter.setShooterSpeed(0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.isAtTarget()){
      //If the drivers are ready to score, score. Otherwise, keep the robot in position but don't shoot. 
      if(m_shouldScore.getAsBoolean()){
        //run the belt to eject the note
        m_shooter.setBeltSpeed(0.75);
      }
      else{
        //if the driver lets go of the trigger, stop the belt until they press it again
        m_shooter.setBeltSpeed(0);
      }
    }
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
    return false;
  }
}
