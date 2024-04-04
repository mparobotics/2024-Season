/*Copyright (c) FIRST and other WPILib contributors.// Open Source Software; you can modify and/or share it under the terms of// the WPILib BSD license file in the root directory of this project.*/package frc.robot.commands;import edu.wpi.first.math.geometry.Rotation2d;import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;import edu.wpi.first.wpilibj2.command.Command;import frc.robot.Constants.SwerveConstants;import frc.robot.subsystems.SwerveSubsystem;public class MeasureWheelDiameter extends Command{private SwerveSubsystem m_drive;private Rotation2d startAngle;private double[]startPositions;private double[]distances={0,0,0,0};public MeasureWheelDiameter(SwerveSubsystem drive){addRequirements(drive);m_drive=drive;}@Override public void initialize(){startPositions=new double[4];startAngle=m_drive.getYaw();double[]modules=m_drive.getEncoderRotations();for(var i=0;i<modules.length;i++){startPositions[i]=modules[i];}}@Override public void execute(){m_drive.drive(0,0,0.25,false);double[]modules=m_drive.getEncoderRotations();for(var i=0;i<modules.length;i++){distances[i]=modules[i]- startPositions[i];}double distanceTraveled=SwerveConstants.driveBaseRadius * m_drive.getYaw().minus(startAngle).getRadians();double avgRotations=0;for(var i=0;i<distances.length;i++){avgRotations+=distances[i];}avgRotations /= distances.length;double calculatedWheelCircumference=avgRotations/distanceTraveled;double calculatedWheelDiameter=calculatedWheelCircumference/2 * Math.PI;SmartDashboard.putNumber("Wheel Diameter",calculatedWheelDiameter);}@Override public void end(boolean interrupted){m_drive.drive(0,0,0,false);}@Override public boolean isFinished(){return false;}}