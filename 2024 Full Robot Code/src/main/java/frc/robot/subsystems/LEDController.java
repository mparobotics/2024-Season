// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.OnboardModuleState;
import frc.robot.Constants.FieldConstants;

/** Add your docs here. */
public class LEDController extends SubsystemBase{
    private final int led_count = 120;
    //define the different segments of the led strip with the LED index of the last led in the segment
    private final int leftBackBar = 15;
    private final int leftFrontBar = 30;
    private final int leftBar = 60;

    private final int rightBar = 90;
    private final int rightFrontBar = 105;
    private final int rightBackBar = 120;

    private Timer m_timer = new Timer();
    //Plug LEDs into PWM port #0
    private AddressableLED m_leds = new AddressableLED(0);
    private AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(led_count);

    private double offset = 0;
    public LEDController(){
        m_leds.setLength(led_count);
        m_leds.start();
        m_timer.start();
    }
    //             .*´^`*.         .*´^`*.         .*´^`*.
    //     `*._.*´         `*._.*´         `*._.*´         `*._.*´      ~~~~~~~~~~~~~~~
    private double wave(double position, double min, double max, double period){
        return Math.cos(position / period * 2 * Math.PI) * (max - min) / 2 + min + (max - min)/2;
    }
    //    /\  /\  /\  /\  /\  /\  /\  /\
    //   /  \/  \/  \/  \/  \/  \/  \/  \           /\/\/\/\/\/\/\/\/\/\/\/\
    private double zigzag(double position, double min, double max, double period){
        return Math.abs(OnboardModuleState.fixedMod(position / period - 0.5, 1) - 0.5) * 2 * (max - min) + min;
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
        return Math.max(-Math.abs((spikePosition - position) / width) + 1,0) * (max - min) + min;
    }


    private void setAll(int r, int g, int b){
        for(var i = 0; i < led_count; i++){
            m_buffer.setRGB(i,r,g,b);
        }
    }
    private void setAllLeft(int r, int g, int b){
        for(var i = 0; i < leftBar; i++){
            m_buffer.setRGB(i,r,g,b);
        }
    }
    private void setAllRight(int r, int g, int b){
        for(var i = leftBar; i < rightBackBar; i++){
            m_buffer.setRGB(i,r,g,b);
        }
    }
    private void setLeftFront(int r, int g, int b){
        for(var i = leftBackBar; i < leftFrontBar; i++){
            m_buffer.setRGB(i,r,g,b);
        }
    }
    private void setRightFront(int r, int g, int b){
        for(var i = rightBar; i < rightFrontBar; i++){
            m_buffer.setRGB(i,r,g,b);
        }
    }
    private void setLeftBack(int r, int g, int b){
        for(var i = 0; i < leftBackBar; i++){
            m_buffer.setRGB(i,r,g,b);
        }
    }
    private void setRightBack(int r, int g, int b){
        for(var i = rightFrontBar; i < rightBackBar; i++){
            m_buffer.setRGB(i,r,g,b);
        }
    }


    

    public void disabledPeriodic(){
        //offset += 1;
        offset = m_timer.get();
        int brightness = (int)wave(offset, 0, 255, 1);
        if(DriverStation.getAlliance().isPresent()){
            if(FieldConstants.isRedAlliance()){
                setAll(brightness,0,0);
            }
            else{
                setAll(0, 0, brightness);
            }
        }
        else{
            brightness *= 0.5;
            setAll(brightness, brightness, brightness);
        }
        m_leds.setData(m_buffer);
    }
    public void autoPeriodic(boolean hasNote){
        offset += 1;
        if(hasNote){
            double brightness = wave(offset, 0, 255, 100);
            setAll((int)brightness,(int)(brightness * 0.1), 0);
        }
        else if(FieldConstants.isRedAlliance()){
            for(var i = 0; i < led_count; i ++){
                m_buffer.setRGB(i, (int)wave(offset, 0, 255, 100),0,0);
            }
        }
        else{
            for(var i = 0; i < led_count; i ++){
                m_buffer.setRGB(i, 0,0,(int)wave(offset, 0, 255, 100));
            }
        }
        m_leds.setData(m_buffer);
    }
    public void teleopPeriodic(boolean hasNote, boolean isInRange, boolean isAtShootingSpeed, boolean isArmTooHigh){
        //offset += 1;
        offset = m_timer.get();
        if(isAtShootingSpeed){
            
            int brightness = (int)wave(offset, 50,255,1);
            setAll(0, brightness, 0);
        }
        else if(isArmTooHigh){
            double brightness = square(offset, 50,255,0.5);
            setAll((int) brightness, (int) (brightness), 0);
        }
        else if(hasNote){
            double brightness = wave(offset, 50,255,1);
            setAll((int) brightness, (int) (brightness * 0.1), 0);
            if(isInRange){
                int B1 = (int)zigzag(offset, 0, 255, 1);
                int B2 = 255 - B1;
                setRightFront(0,B1 ,B1);
                setLeftBack(0,B1 ,B1 );
                setLeftFront(0, B2,B2);
                setRightBack(0, B2, B2);
            }
        }
        else if (FieldConstants.isRedAlliance()){
            for(var i = 0; i < led_count; i ++){
                if((i + offset * 5)% 6 >= 3){
                    m_buffer.setRGB (i, 200,0,0);
                }
                else{ 
                    m_buffer.setRGB (i,130,40,130);
                }
            }
        } else {
            for(var i = 0; i < led_count; i ++){
                if((i + offset * 5)% 6 >= 3){
                    m_buffer.setRGB (i, 0,0,200);
                }
                else{ 
                    m_buffer.setRGB (i,150,40,150);
                }
            }
        }
        m_leds.setData(m_buffer);
    }
    @Override
    public void periodic(){
        
    }
}
