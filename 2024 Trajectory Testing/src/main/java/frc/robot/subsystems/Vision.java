// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;


/** Add your docs here. */
public class Vision {
    public static NetworkTable getLimelight(){
        return NetworkTableInstance.getDefault().getTable("limelight");
    }
    public static NetworkTable getLimelight(String name){
        String Name = name;
        if(Name == "" || Name == null){
            Name = "limelight";
        }
        return NetworkTableInstance.getDefault().getTable(Name);
    }
    
    public static boolean canSeeTarget(){
        return getLimelight().getEntry("tv").getDouble(0) == 1;
    }
    public static double getTx(){
        return getLimelight().getEntry("tx").getDouble(0);
    }
    public static double getTargetID(){
        return getLimelight().getEntry("tid").getDouble(0);
    }
    public static double[] getBotPoseArray(){
        return getLimelight().getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }
    public static double[] getCameraPoseArray(){
        return  getLimelight().getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }
    public static Translation2d get2dOffset(){
        double[] transform = getCameraPoseArray();
        return new Translation2d(transform[0],transform[1]);
    }
    public static Pose2d getBotPose(){
        double[] poseArray = getBotPoseArray();
        return new Pose2d(poseArray[0],poseArray[1],Rotation2d.fromDegrees(poseArray[5]));
    }
    public static double getLatency(){
        return Timer.getFPGATimestamp() - getBotPoseArray()[6]/1000.0;
    }
    public static double getDistanceToTarget(){
        double[] transform = getCameraPoseArray();
        return Math.sqrt(transform[0] * transform[0] + transform[1] * transform[1] + transform[2] * transform[2]);
    }
}
