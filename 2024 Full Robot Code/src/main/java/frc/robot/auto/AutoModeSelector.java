// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoModeSelector {
    private ArmSubsystem m_arm;
    private ShooterSubsystem m_shooter;
    private IntakeSubsystem m_intake;
    private SwerveSubsystem m_drive;


    private SendableChooser<AutoMode> autoChoices;
    public enum AutoMode{
        NO_AUTO,
        TEST,
        TWO_NOTE_CENTER,
        FIVE_NOTE,
        TWO_NOTE,
        FOUR_NOTE,
    }
    public AutoModeSelector(ArmSubsystem arm, ShooterSubsystem shooter, IntakeSubsystem intake, SwerveSubsystem drive){
        m_arm = arm;
        m_shooter = shooter;
        m_intake = intake;
        m_drive = drive;
    }
    public void showOptions(){
        autoChoices = new SendableChooser<AutoMode>();
        for(AutoMode mode: AutoMode.values()){
            autoChoices.addOption(mode.toString(), mode);
        }
        autoChoices.setDefaultOption("Do Nothing", AutoMode.NO_AUTO);
        SmartDashboard.putData("Auto Mode Selector", autoChoices);
        
    }
    public Command getSelectedAuto(){
        return getAuto(autoChoices.getSelected());
    }
    public Command getAuto(AutoMode mode){
        switch(mode){
            case NO_AUTO:
                return null;
            case TEST:
                return m_drive.followPathFromFile("testPath01");
            case TWO_NOTE_CENTER:
                return new OneCenterNote(m_drive, m_intake, m_shooter, m_arm);
            case FIVE_NOTE:
                return new FiveNoteAuto(m_drive, m_intake, m_shooter, m_arm);
            case TWO_NOTE:
                return new TwoNoteAuto(m_drive, m_intake, m_shooter, m_arm);
            case FOUR_NOTE:
                return new FourNoteAuto(m_drive, m_intake, m_shooter, m_arm);





        }
        return null;
    }
    
}
