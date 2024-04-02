// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootLeaveAuto extends SequentialCommandGroup {
  private SwerveSubsystem m_drive;
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;
  
  /** An auto that shoots a preload and backs up past the auto line */
  public ShootLeaveAuto(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;

    m_shooter = shooter;
    m_arm = arm;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_drive.startAutoAt(0.71, 4.43, -60),
      m_shooter.spinUpShooterCommand(),
      m_arm.setArmSetpointCommand(25),
      new Shoot(m_shooter, () -> true),

      m_drive.followPathFromFile("Leave"),
      m_shooter.stopShooterCommand()
    );
  }
}
