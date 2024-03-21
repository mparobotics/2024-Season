// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.ShooterSubsystem;


public class IntakeOveride extends Command {
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;


  /** Runs the intake but ignores the beam break (for cases when the beam break fails or gets blocked by something)*/
  public IntakeOveride(IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter) {
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
    
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setBeltSpeed(0.75);
    m_intake.runIntake(1);
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
    return false;
  }
}
