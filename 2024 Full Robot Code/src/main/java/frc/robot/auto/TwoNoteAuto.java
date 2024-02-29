// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAuto extends SequentialCommandGroup {
  private SwerveSubsystem m_drive;
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;

  /** A simple Two Note Auto */
  public TwoNoteAuto(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_arm = arm;
 

    addCommands(
      new InstantCommand(() -> m_drive.resetOdometry(FieldConstants.flipPoseForAlliance(new Pose2d(1.35,5.49,Rotation2d.fromDegrees(0))))),
      m_arm.setArmSetpointCommand(() -> 25),
      //Spin up the shooter wheels. We keep them running for the entirety of auto
      m_shooter.spinUpShooterCommand(),
      //shoot the preload
      new Shoot(m_shooter, () -> true),

      m_arm.setArmSetpointCommand(() -> 19.8),
      //drive to the note that's next to the stage
      new ParallelCommandGroup(m_drive.followPathFromFile("SW3"), new Intake(m_intake, m_arm, m_shooter)),
      m_arm.setArmSetpointCommand(() -> 28),
      //drive back to the speaker
      m_drive.followPathFromFile("W3S"),
      //Shoot the second note
      new Shoot(m_shooter, () -> true)

    );
  }
  public Pose2d getStartingPose(){
    return new Pose2d();
  }
}
