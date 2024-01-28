// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class Vision {
    public NetworkTable getLimelight(){
        return NetworkTableInstance.getDefault().getTable("limelight");
    }
    public boolean canSeeTarget(){
        return getLimelight().getEntry("tv").getDouble(0) == 1;
    }
    public double getTx(){
        return getLimelight().getEntry("tx").getDouble(0);
    }
    public double getTargetID(){
        return getLimelight().getEntry("tid").getDouble(0);
    }
    public double[] getBotPoseArray(){
        if(DriverStation.getAlliance().get() == Alliance.Red)   
            return getLimelight().getEntry("botpose_wpired").getDoubleArray(new double[6]);
        else{
            return getLimelight().getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }
    }
    public Translation2d get2dOffset(){
        double[] transform = getLimelight().getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        return new Translation2d(transform[0],transform[1]);
    }
    public  Pose2d getBotPose(){
        double[] poseArray = getBotPoseArray();
        return new Pose2d(poseArray[0],poseArray[1],Rotation2d.fromDegrees(poseArray[5]));
    }
    public double getLatency(){
        return Timer.getFPGATimestamp() - getBotPoseArray()[6]/1000.0;
    }
}
