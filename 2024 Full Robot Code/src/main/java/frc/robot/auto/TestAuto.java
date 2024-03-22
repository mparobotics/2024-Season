// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;



import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  private SwerveSubsystem m_drive;
  private IntakeSubsystem m_intake;

  /** A simple Two Note Auto */
  public TestAuto(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;
    m_intake = intake;

 

    addCommands(
      
      m_drive.startAutoAt(1.35, 5.49, 0),
    
      //Spin up the shooter wheels. We keep them running for the entirety of auto
     m_intake.IntakeControlCommand(() -> 1),
    

      
      //drive to the note that's next to the stage
      new ParallelCommandGroup(m_drive.followPathFromFile("SW3")), 
      //start moving the arm to the shooting angle
   
      //drive back to the speaker
      m_drive.followPathFromFile("W3S"),
     

      //drive to the middle of the 3 close notes
      new ParallelCommandGroup(m_drive.followPathFromFile("SW2")), 
      //start moving the arm to the shooting angle
    
      //drive back to the speaker
      m_drive.followPathFromFile("W2S"),
     

      
      new ParallelCommandGroup(m_drive.followPathFromFile("SW1")),
      //start moving the arm to the shooting angle
    
      //drive back to the speaker
      m_drive.followPathFromFile("W1S"),
      //drive to center note2 
      m_drive.followPathFromFile("SC2"),
      //drive to speaker from center note 2
      m_drive.followPathFromFile("C2S")

    );
  }
}
