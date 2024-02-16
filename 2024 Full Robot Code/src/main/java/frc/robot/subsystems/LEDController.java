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

    
    public LEDController(){
        leds.setLength(led_count);
        leds.start();
    }
    public void setAll(int r, int g, int b){
        for(var i = 0; i < led_count; i++){
            buffer.setRGB(i,r,g,b);
        }
        leds.setData(buffer);
    }
    
    @Override
    public void periodic(){
        
        
    }



}
