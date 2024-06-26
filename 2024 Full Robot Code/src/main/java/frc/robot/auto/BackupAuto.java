// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;



import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackupAuto extends SequentialCommandGroup {
  private SwerveSubsystem m_drive;
  private ShooterSubsystem m_shoot;
  private ArmSubsystem m_arm;
  
  /** An Auto that just backs up past the auto line */
  public BackupAuto(SwerveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    m_drive = drive;
    m_shoot = shooter; 
    m_arm = arm;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
  
     m_shoot.spinUpShooterCommand(), //spins up shooter
     m_arm.setArmSetpointCommand(25), //sets arm to 25deg
     new Shoot(shooter, () -> true), //makes the robot shoot () -> makes it a boolean supplier
     m_shoot.stopShooterCommand(),
     new WaitCommand(10),
     m_drive.backupCommand() 
     
    );
  }
}
