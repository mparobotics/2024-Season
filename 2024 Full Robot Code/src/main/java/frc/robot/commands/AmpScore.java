// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.ShooterSubsystem;


public class AmpScore extends Command {
  private ArmSubsystem m_arm;
  private LEDController m_led;
  private ShooterSubsystem m_shooter;
  
  private BooleanSupplier m_shouldScore;

  private Timer m_timer;
  private boolean hasStartedScoring = false;
  
  /** Command to score a note in the AMP*/
  public AmpScore(ArmSubsystem arm,LEDController led, ShooterSubsystem shooter, BooleanSupplier shouldScore) {
    addRequirements(arm);
    addRequirements(led);
    addRequirements(shooter);
    m_arm = arm;
    m_led = led;
    m_shooter = shooter;
    m_shouldScore = shouldScore;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setToAmpAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.isAtTarget()){
      //If we want to score, score. Otherwise, keep the robot ready but don't shoot. 
      if(m_shouldScore.getAsBoolean()){

        //display shooting pattern
        m_led.setAll(255,0,0);
        //run the belt and the shooter (at low speed) to eject the note
        m_shooter.setBeltSpeed(1);
        m_shooter.setShooterSpeed(0.4);
        //reset and start the shooter clock
        m_timer.reset();
        m_timer.start();

        //we have now started scoring
        hasStartedScoring = true;
      }
      else{
        //display ready to score pattern
        m_led.setAll(0,255,0);
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
    return hasStartedScoring && m_timer.get() > ShooterConstants.shootTimeSeconds;
  }
}