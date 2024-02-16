// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoModeSelector {
    private ArmSubsystem m_arm;
    private ShooterSubsystem m_shooter;
    private IntakeSubsystem m_intake;
    private SwerveSubsystem m_drive;
    private LEDController m_led;

    private SendableChooser<AutoMode> autoChoices;
    public enum AutoMode{
        NOAUTO,
        TEST
    }
    public AutoModeSelector(ArmSubsystem arm, ShooterSubsystem shooter, IntakeSubsystem intake, SwerveSubsystem drive, LEDController led){
        m_arm = arm;
        m_shooter = shooter;
        m_intake = intake;
        m_drive = drive;
        m_led = led;
    }
    public void showOptions(){
        autoChoices = new SendableChooser<AutoMode>();
        for(AutoMode mode: AutoMode.values()){
            autoChoices.addOption(mode.toString(), mode);
        }
        autoChoices.setDefaultOption("Do Nothing", AutoMode.NOAUTO);
        SmartDashboard.putData(autoChoices);
    }
    public Command getAuto(AutoMode mode){
        switch(mode){
            case NOAUTO:
                return null;
            case TEST:
                return m_drive.followPathFromFile("testPath01");



        }
        return null;
    }
    
}
