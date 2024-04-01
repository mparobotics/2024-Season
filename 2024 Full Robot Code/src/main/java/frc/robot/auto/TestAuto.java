// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;




import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake;
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
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;
  /** Auto mode for testing new features (NOT FOR COMPETITION) */
  public TestAuto(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_arm = arm;

 

    addCommands(
      
      m_drive.startAutoAt(1.89, 5.55, 0),

      //score the C2 center note
      new ParallelDeadlineGroup(
        //drive out to the note and back to the speaker
        m_drive.followPathFromFile("SC2").andThen(m_drive.followPathFromFile("C2S")),
        //run the intake until we have the note, then set the arm to the shooting position
        new Intake(m_intake, m_arm, m_shooter).andThen(m_arm.setArmSetpointCommand(30))
      )
    );
  }
}
