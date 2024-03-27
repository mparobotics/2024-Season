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


public class CenterNoteAuto extends SequentialCommandGroup {
  private SwerveSubsystem m_drive;
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;

  /**Scores preload and Gets the two center notes on the source side of the field  */
  public CenterNoteAuto(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_arm = arm;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        m_drive.startAutoAt(0.71, 4.43, -60),
        //set arm to shooting position
        m_arm.setArmSetpointCommand(() -> 25),
        //spins up shooter and shoots,
        m_shooter.spinUpShooterCommand()
      ),
      new Shoot(m_shooter, () -> true),
      
      //moves to the C5 center note and intakes
      new ParallelCommandGroup(m_drive.followPathFromFile("SC5"), new Intake(m_intake, m_arm, m_shooter)),
      //set arm to shooting position
      m_arm.setArmSetpointCommand(() -> 33),
      //goes back to the speaker
      m_drive.followPathFromFile("C5S"),

      
      //shoots C5 center note
      new Shoot(m_shooter, () -> true),

      //moves to the C4 center note and intakes
      new ParallelCommandGroup(m_drive.followPathFromFile("SC4"),new Intake(m_intake, m_arm, m_shooter)),
      //goes back to the speaker
      m_drive.followPathFromFile("C4S"),
      //set arm to shooting angle
      m_arm.setArmSetpointCommand(() -> 33),
      //shoots the C4 center note
      new Shoot(m_shooter, () -> true)
    );
  }
}
