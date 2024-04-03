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

/** A class for selecting an auto mode from a list of options */
public class AutoModeSelector {
    private ArmSubsystem m_arm;
    private ShooterSubsystem m_shooter;
    private IntakeSubsystem m_intake;
    private SwerveSubsystem m_drive;

    //A dropdown menu to select an auto mode
    private SendableChooser<Command> autoChoices;
    //All possible auto choices
    public enum AutoMode{
        NO_AUTO,
        TWO_NOTE_CENTER,
        FOUR_NOTE,
        FIVE_NOTE,
        SWEEP,
        SWEEP_AND_RECOVER,
        AMP,
        JUST_SHOOT,
        SHOOT_AND_LEAVE,
        AMP_SIDE_FOUR_NOTE,
        TEST_AUTO,
        BACKUP_AUTO
    }
    //default to a shoot + back up auto so we still get points if we forget to select an auto
    private AutoMode defaultAuto = AutoMode.BACKUP_AUTO;
    //An autoModeSelector feeds all the subsytems to each auto command
    public AutoModeSelector(ArmSubsystem arm, ShooterSubsystem shooter, IntakeSubsystem intake, SwerveSubsystem drive){
        m_arm = arm;
        m_shooter = shooter;
        m_intake = intake;
        m_drive = drive;
    }
    //send the menu to networkTables
    public void showOptions(){
        autoChoices = new SendableChooser<Command>();
        //Add each auto choice to the list
        for(AutoMode mode: AutoMode.values()){
            autoChoices.addOption(mode.toString(), getAuto(mode));
        }
        //set a default auto mode
        autoChoices.setDefaultOption(defaultAuto.toString(), getAuto(defaultAuto));
        SmartDashboard.putData("Auto Mode Selector", autoChoices);
       
    }
    //return the selected auto command
    public Command getSelectedAuto(){
        return autoChoices.getSelected();
    }
    //get the corresponding auto command for a given AutoMode type
    public Command getAuto(AutoMode mode){
        if(mode == null){
            return null;
        }
        switch(mode){
            case NO_AUTO:
                return null;
            case TWO_NOTE_CENTER:
                return new CenterNoteAuto(m_drive, m_intake, m_shooter, m_arm);
            case FOUR_NOTE:
                return new FourNoteAuto(m_drive, m_intake, m_shooter, m_arm);
            case FIVE_NOTE:
                return new FiveNoteAuto(m_drive, m_intake, m_shooter, m_arm);
            case AMP:
                return new SpeakerAmpAuto(m_drive, m_intake, m_shooter, m_arm);
            case SWEEP:
                return new SweepAuto(m_drive, m_intake, m_shooter, m_arm);
            case SWEEP_AND_RECOVER:
                return new SweepAndRecoverAuto(m_drive, m_intake, m_shooter, m_arm);
            case JUST_SHOOT:
                return new ShootAuto(m_drive, m_intake, m_shooter, m_arm);
            case SHOOT_AND_LEAVE:
                return new ShootLeaveAuto(m_drive, m_intake, m_shooter, m_arm);
            case AMP_SIDE_FOUR_NOTE:
                return new AmpSideFourAuto(m_drive, m_intake, m_shooter, m_arm);
            case TEST_AUTO:
                return new TestAuto(m_drive,m_intake, m_shooter, m_arm);
            case BACKUP_AUTO:
                return new BackupAuto(m_drive,m_intake, m_shooter, m_arm);
        }
        return null;
    }
    
}
