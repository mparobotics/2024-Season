// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;




import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.SweepAutoIntakeControl;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SweepAuto extends SequentialCommandGroup {
  private SwerveSubsystem m_drive;
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;

  /**An auto that sweeps across the centerline and tosses all 5 notes towards our side */
  public SweepAuto(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_arm = arm;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_drive.startAutoAt(1.41, 1.59, 90),
      
      m_arm.armDownCommand(),
      //simultaneously follows the path and executes the intake and shooter controlling commands
      new ParallelCommandGroup(
        //start driving the sweep path as soon as possible. We need to be the first ones out to the center
        m_drive.followPathFromFile("Sweep"),
        new SequentialCommandGroup(
          //spit the preload out in our wing, but do it while already moving so we don't waste time
          new ReverseIntake(m_intake, m_shooter).withTimeout(2),
          //count out 5 notes fed through the system and hold onto the 5th one
          new SweepAutoIntakeControl(m_intake, m_shooter),
          //spin up the shooter so we're ready to shoot when we get to the 
          m_shooter.spinUpShooterCommand()
        )
      ),
      //move the arm to the correct shooting angle
      m_arm.setArmSetpointCommand(() -> 51),
      //shoot the 5th center note
      new Shoot(shooter, () -> true),
      //stop the shooter
      m_shooter.stopShooterCommand()
    
        
    );

     
  }
}
