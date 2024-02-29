// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.OnboardModuleState;
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
    //             .*´^`*.         .*´^`*.         .*´^`*.
    //     `*._.*´         `*._.*´         `*._.*´         `*._.*´      ~~~~~~~~~~~~~~~
    private double wave(double position, double min, double max, double period){
        return Math.sin(position * period * 2 * Math.PI) * (max - min) / 2 + min + (max - min)/2;
    }
    //    /\  /\  /\  /\  /\  /\  /\  /\
    //   /  \/  \/  \/  \/  \/  \/  \/  \           /\/\/\/\/\/\/\/\/\/\/\/\
    private double zigzag(double position, double min, double max, double period){
        return Math.abs(OnboardModuleState.fixedMod(position / period - 0.5, 1) - 0.5) * (max - min) + min;
    }
    //    / / / / / / / / / / / / / / / / / / / / 
    //   / / / / / / / / / / / / / / / / / / / /        ///////////
    private double sawtooth(double position, double min, double max, double period){
        return OnboardModuleState.fixedMod(position / period, 1) * (max - min) + min;
    }
    //       ____    ____    ____    ____    ____               This is a square -> ☐
    //   ____    ____    ____    ____    ____      <- That's not a square           
    private double square(double position, double min, double max, double period){
        return OnboardModuleState.fixedMod(position / period, 1) > 0.5? max : min;
    }
    //                           /\  
    //__________________________/  \__________________________         _/\_
    private double spike(double position, double spikePosition, double width, double min, double max){
        return Math.max(-Math.abs((spikePosition - position + max - min) / width),0) + min;
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
                        Buffer.setRGB (i, 200,0,0);
                    }
                    else{ 
                        Buffer.setRGB (i,130,40,130);
                    }
                }
            } else {
                offset += 0.5;
                for(var i = 0; i < led_count; i ++){
                    if((i + offset)% 6 >= 3){
                        Buffer.setRGB (i, 0,0,200);
                    }
                    else{ 
                        Buffer.setRGB (i,150,40,150);
                    }
                }
            }
            leds.setData(Buffer);

        });
        
    }
    @Override
    public void periodic(){
        if (DriverStation.isDisabled()) {
            offset += 1;
            if(FieldConstants.isRedAlliance()){
                setAll((int)wave(offset, 0, 200, 100),0,130);
            }
            else{
                setAll(0, 0, (int)wave(offset, 0, 255, 100));
            }
            
        }
        
    }



}
