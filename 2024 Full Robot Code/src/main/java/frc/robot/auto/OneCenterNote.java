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
public class OneCenterNote extends SequentialCommandGroup {
  private SwerveSubsystem m_drive;
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;

  /** Creates a new OneCenterNote. */
  public OneCenterNote(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_arm = arm;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_drive.resetOdometry(FieldConstants.flipPoseForAlliance(new Pose2d(0.71,4.43,Rotation2d.fromDegrees(-60))))),
      //spins up shooter and shoots,
      m_shooter.shooterControlCommand(() -> 1,() -> 0),
      new Shoot(m_shooter, () -> true),
      
      //moves to the C5 center note and intakes
      new ParallelCommandGroup(m_drive.followPathFromFile("SC5"), new Intake(m_intake, m_arm, m_shooter)),
      //goes back to the speaker
      m_drive.followPathFromFile("C5S"),
      //shoots
      new Shoot(m_shooter, () -> true),

      //moves to the C4 center note and intakes
      new ParallelCommandGroup(m_drive.followPathFromFile("SC4") ,new Intake(m_intake, m_arm, m_shooter)),
      //goes back to the speaker
      m_drive.followPathFromFile("C4S"),
      //shoots
      new Shoot(m_shooter, () -> true)
    );
  }
}
