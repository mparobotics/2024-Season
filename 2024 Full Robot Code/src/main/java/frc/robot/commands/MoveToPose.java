// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.OnboardModuleState;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPose extends Command {
  SwerveSubsystem m_drive;
  Supplier<Pose2d> m_goalPoseSupplier;


  private ProfiledPIDController xController = new ProfiledPIDController(AutoConstants.translation_kP, AutoConstants.translation_kI, AutoConstants.translation_kD, AutoConstants.autoAlignXYConstraints);
  private ProfiledPIDController yController = new ProfiledPIDController(AutoConstants.translation_kP, AutoConstants.translation_kI, AutoConstants.translation_kD, AutoConstants.autoAlignXYConstraints);
  private ProfiledPIDController angleController = new ProfiledPIDController(AutoConstants.rotation_kP, AutoConstants.rotation_kI, AutoConstants.rotation_kD, AutoConstants.autoAlignRConstraints);

  /** Command to drive to a target pose using a PID contoller*/
  public MoveToPose(SwerveSubsystem drive, Supplier<Pose2d> goalPose) {
    addRequirements(drive);
    m_drive = drive;
    m_goalPoseSupplier = goalPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();
    double currentDirection = currentPose.getRotation().getDegrees();
    Pose2d targetPose = m_goalPoseSupplier.get();
      
      
    double targetDirection = OnboardModuleState.smolOptimize180(currentDirection,targetPose.getRotation().getDegrees());

    double xSpeed = xController.calculate(currentPose.getX(),targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(),targetPose.getY());
    double rSpeed = angleController.calculate(currentDirection, targetDirection);

    SmartDashboard.putNumber("x output", xSpeed);
    SmartDashboard.putNumber("y output", ySpeed);
      
    m_drive.drive(xSpeed, ySpeed, rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
