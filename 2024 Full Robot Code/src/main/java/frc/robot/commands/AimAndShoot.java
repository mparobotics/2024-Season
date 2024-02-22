// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;






import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.ShooterSubsystem;


public class AimAndShoot extends Command {
  
  private ArmSubsystem m_arm;
  private LEDController m_led;
  private ShooterSubsystem m_shooter;
  private DoubleSupplier m_distance;

  private BooleanSupplier m_shouldShoot;
  private boolean hasStartedShooting = false;

  private Timer m_timer;
  public AimAndShoot(ArmSubsystem arm,LEDController led, ShooterSubsystem shooter, DoubleSupplier shootingDistance, BooleanSupplier shouldShoot) {
    addRequirements(arm);
    addRequirements(led);
    addRequirements(shooter);
    m_arm = arm;
    m_led = led;
    m_shooter = shooter;
    m_distance = shootingDistance;
    m_shouldShoot = shouldShoot;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    //move the arm towards the target angle (automatically calculated)
    m_arm.setAngleForShootingDistance(m_distance.getAsDouble());
    //spin the wheels to the correct speed
    m_shooter.spinUpShooter();
    
    //we have not started shooting yet
    hasStartedShooting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //check if the shooter is ready to shoot (arm is in position and wheels are spinning fast enough)
    if(m_arm.isAtTarget() && m_shooter.isAtShootingSpeed()){
      //If we want to shoot, shoot. Otherwise, keep the shooter ready but don't shoot. this is useful if you are being defended on and have to move around while preparing to shoot
      if(m_shouldShoot.getAsBoolean()){

        //display shooting pattern
        m_led.setAll(255,0,0);
        if(!hasStartedShooting){
          //feed the note into the shooter wheels by running the belts
          m_shooter.setBeltSpeed(1);
          //reset and start the shooter clock
          m_timer.reset();
          m_timer.start();
          //we have now started shooting
          hasStartedShooting = true;
        }
      }
      else{
        //display ready to shoot pattern
        m_led.setAll(0,255,0);
      }
    }
    else{
      //display preparing to shoot pattern
      m_led.setAll(0,0,255);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Ends the command when we have shot the note, with extra delay time added to allow the note to leave the shooter before we slow down the wheels
  @Override
  public boolean isFinished() {
    return hasStartedShooting && m_timer.get() > ShooterConstants.shootTimeSeconds;
  }
}