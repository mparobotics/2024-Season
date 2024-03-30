// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourNoteAuto extends SequentialCommandGroup {
  private SwerveSubsystem m_drive;
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;

  /** Score preload + 3 close notes */
  public FourNoteAuto(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_arm = arm;
 

    addCommands(
      new ParallelCommandGroup(
        m_drive.startAutoAt(1.35, 5.49, 0),
        m_arm.setArmSetpointCommand(25),
        //Spin up the shooter wheels. We keep them running for the entirety of auto
        m_shooter.spinUpShooterCommand()
      ),
      //shoot the preload
      new Shoot(m_shooter, () -> true),

      
      //drive to the note that's next to the stage
      new ParallelCommandGroup(m_drive.followPathFromFile("SW3"), new Intake(m_intake, m_arm, m_shooter)),
      //start moving the arm to the shooting angle
      m_arm.setArmSetpointCommand(30),
      //drive back to the speaker
      m_drive.followPathFromFile("W3S"),
      //Shoot the second note
      new Shoot(m_shooter, () -> true),

      //drive to the middle of the 3 close notes
      new ParallelCommandGroup(m_drive.followPathFromFile("SW2"), new Intake(m_intake, m_arm, m_shooter)),
      //start moving the arm to the shooting angle
      m_arm.setArmSetpointCommand(30),
      //drive back to the speaker
      m_drive.followPathFromFile("W2S"),
      //shoot the third note
      new Shoot(m_shooter, () -> true),

      
      new ParallelCommandGroup(m_drive.followPathFromFile("SW1"), new Intake(m_intake, m_arm, m_shooter)),
      //start moving the arm to the shooting angle
      m_arm.setArmSetpointCommand(30),
      //drive back to the speaker
      m_drive.followPathFromFile("W1S"),
      //shoot the final note
      new Shoot(m_shooter, () -> true)

    );
  }
}
