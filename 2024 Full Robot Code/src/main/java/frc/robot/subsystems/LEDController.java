// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LEDController extends SubsystemBase{
    private final int led_count = 60;
    //Plug LEDs into PWM port #0
    private AddressableLED leds = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(led_count);

    private LedPattern currentPattern;
    public enum LedPattern{
        TEST,
        TELEOP_DRIVING,
        HAS_NOTE,

        PREPARING_SHOOT,
        PREPARING_AMP,
        SHOOTING,
        AMP,


    }
    public LEDController(){
        leds.setLength(led_count);
        leds.start();
        
    }
    public void setPatternMode(LedPattern mode){
        currentPattern = mode;
    }
    public void green(){
        for(var i = 0; i < led_count; i++){
            buffer.setRGB(i,0,255,0);
        }
    }
    
    @Override
    
    public void periodic(){
        switch (currentPattern){
            case TEST:

            case TELEOP_DRIVING:

            case HAS_NOTE:

            case PREPARING_AMP:
            case PREPARING_SHOOT:

            case AMP:

            case SHOOTING:

          

             

        }
        green();
        leds.setData(buffer);
    }



}
