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
import frc.robot.Constants.VisionConstants;


/** Add your docs here. */
public class Vision {
    public static NetworkTable getAprilTagDetector(){
        return NetworkTableInstance.getDefault().getTable("limelight-b");
    }
    public static NetworkTable getNoteDetector(){
        return NetworkTableInstance.getDefault().getTable("limelight-a");
    }
    
    public static boolean canSeeAprilTag(){
        return getAprilTagDetector().getEntry("tv").getDouble(0) == 1;
    }
    public static boolean canSeeNote(){
        return getNoteDetector().getEntry("tv").getDouble(0) == 1;
    }
    public static double getNoteXAngleOffset(){
        return getNoteDetector().getEntry("tx").getDouble(0);
    }
    public static double getNoteYAngleOffset(){
        return getNoteDetector().getEntry("ty").getDouble(0);
    }
    public static Translation2d getRelativeNoteLocation(){
        
        double distance = Math.tan(getNoteYAngleOffset() - VisionConstants.noteLimelightAngle)/VisionConstants.noteLimelightTz;

        double xoffset = Math.cos(getNoteXAngleOffset()) * distance + VisionConstants.noteLimelightTx;
        double yoffset = Math.sin(getNoteXAngleOffset()) * distance + VisionConstants.noteLimelightTy;
       
        return new Translation2d(xoffset,yoffset);
    }
    public static double getTagID(){
        return getAprilTagDetector().getEntry("tid").getDouble(0);
    }
    public static double[] getBotPoseArray(){
        return getAprilTagDetector().getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }
    public static double[] getCameraPoseArray(){
        return  getAprilTagDetector().getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
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
