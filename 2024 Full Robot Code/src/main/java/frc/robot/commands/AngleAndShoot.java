// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;






import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

import frc.robot.subsystems.ShooterSubsystem;


public class AngleAndShoot extends Command {
  
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  private DoubleSupplier m_angle;

  private BooleanSupplier m_shouldShoot;
  private boolean hasStartedShooting = false;

  private Timer m_timer = new Timer();
  /*Aim at a specified angle and shoot a note */
  public AngleAndShoot(ArmSubsystem arm, ShooterSubsystem shooter, DoubleSupplier angle, BooleanSupplier shouldShoot) {
    addRequirements(arm, shooter);

    m_arm = arm;
    m_shooter = shooter;
    m_angle = angle;
    m_shouldShoot = shouldShoot;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    
    //spin the wheels to the correct speed
    m_shooter.spinUpShooter();
    
    //we have not started shooting yet
    hasStartedShooting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //move the arm towards the target angle (supplied in the constructor)
    m_arm.setTarget(m_angle.getAsDouble());
    //check if the shooter is ready to shoot (arm is in position and wheels are spinning fast enough)
    if(m_arm.isAtTarget() && m_shooter.isAtShootingSpeed()){
      //If we want to shoot, shoot. Otherwise, keep the shooter ready but don't shoot. this is useful if you are being defended on and have to move around while preparing to shoot
      if(m_shouldShoot.getAsBoolean()){

        //feed the note into the shooter wheels by running the belts
        m_shooter.setBeltSpeed(1);
        
        if(!hasStartedShooting){
          
          //reset and start the shooter clock
          m_timer.reset();
          m_timer.start();
          //we have now started shooting
          hasStartedShooting = true;
        }
      }
      else{
        m_shooter.setBeltSpeed(0);
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //put the arm back down when we're done shooting (don't smack the stage!)
    m_arm.setToHandoffAngle();
    //stop the shooter and indexer
    m_shooter.stopShooting();
  }

  // Ends the command when we have shot the note, with extra delay time added to allow the note to leave the shooter before we slow down the wheels
  @Override
  public boolean isFinished() {
    return false;
  }
}
