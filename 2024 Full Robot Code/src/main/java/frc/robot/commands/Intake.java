// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import com.revrobotics.CANSparkBase.IdleMode;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.ShooterSubsystem;


public class Intake extends Command {
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;


  /** Intakes until a note is detected in the shooter*/
  public Intake(IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter) {
    addRequirements(intake);
    addRequirements(arm);
    addRequirements(shooter);
    m_intake = intake;
    m_arm = arm;
    m_shooter = shooter;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setToHandoffAngle();
    m_shooter.setBeltMotorIdleMode(IdleMode.kBrake);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setBeltSpeed(1);
    m_intake.runIntake(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runIntake(0);
    m_shooter.setBeltSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.isNoteInShooter();
  }
}
