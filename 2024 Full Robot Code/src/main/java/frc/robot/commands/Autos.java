// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of drive project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class Autos {
    public static enum AutoChoices{
        SIXNOTE,
        CENTERLINE,
        TAKE_TWO,
        UNDERSTAGE,
        FOURNOTE,
        JUST_LEAVE,

    }
    //Config for PathPlanner. contains trajectory PID constants and other drivebase data
    public static final HolonomicPathFollowerConfig pathConfig = new HolonomicPathFollowerConfig( 
        new PIDConstants(5.0, 0.00001, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0005, 0.001), // Rotation PID constants
        SwerveConstants.maxSpeed, // Max module speed, in m/s
        SwerveConstants.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the Pathplanner API for the options here
    );
    public void useDrivetrain(SwerveSubsystem drive){
        AutoBuilder.configureHolonomic(
            drive::getPose,
            drive::resetOdometry,
            drive::getRobotRelativeSpeed,
            drive::closedLoopDrive,
            pathConfig,
            () -> (DriverStation.getAlliance().get() ==  Alliance.Red),
            drive
        );
    }
    Pose2d waypoint(double x, double y, double r){
        return new Pose2d(x,y, Rotation2d.fromDegrees(r));
    }
    
    public static SequentialCommandGroup getAuto(AutoChoices auto){
        switch(auto){
            case SIXNOTE:
                break;
            case CENTERLINE:
                break;
            case FOURNOTE:
                break;
            case JUST_LEAVE:
                break;
            case TAKE_TWO:
                break;
            case UNDERSTAGE:
                break;
        }

        return new SequentialCommandGroup();      

    }
    
}
