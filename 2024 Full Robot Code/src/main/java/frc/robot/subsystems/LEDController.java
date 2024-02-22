// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

/** Add your docs here. */
public class LEDController extends SubsystemBase{
    private final int led_count = 120;
    //Plug LEDs into PWM port #0
    private AddressableLED leds = new AddressableLED(0);
    private AddressableLEDBuffer Buffer = new AddressableLEDBuffer(led_count);

    private double offset = 0;
    public LEDController(){
        leds.setLength(led_count);
        leds.start();
    }

    public void setAll(int r, int g, int b){
        for(var i = 0; i < led_count; i++){
            Buffer.setRGB(i,r,g,b);
        }
        leds.setData(Buffer);
    }
    
    public Command idleLedPattern(){
        return runOnce(() -> {
            if (FieldConstants.isRedAlliance()){
                offset += 0.5;
                for(var i = 0; i < led_count; i ++){
                    if((i + offset)% 6 >= 3){
                        Buffer.setRGB (i, 255,0,0);
                    }
                    else{ 
                        Buffer.setRGB (i,0,255,0);
                    }
                }
            } else {
                offset += 0.5;
                for(var i = 0; i < led_count; i ++){
                    if((i + offset)% 6 >= 3){
                        Buffer.setRGB (i, 0,0,255);
                    }
                    else{ 
                        Buffer.setRGB (i,0,255,0);
                    }
                }
            }
            leds.setData(Buffer);

        });
        
    }
    @Override
    public void periodic(){
        
    }



}
