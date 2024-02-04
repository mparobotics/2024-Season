// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;


//A Subsystem to control a single NEO motor
public class MotorSubsystem extends SubsystemBase {

  //Create a SparkMAX motor controller
  private final CANSparkMax testMotor = new CANSparkMax(53, MotorType.kBrushless);
  private final RelativeEncoder encoder = testMotor.getEncoder();
  
  private final int led_count = 60;
  private double offset = 0;
  private final AddressableLED leds = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(led_count);

  private final DigitalInput beamBreak = new DigitalInput(0);
  
  

 

  

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    leds.setLength(led_count);
    leds.start();
    
    
    
  }
  //A command that sets the motor to a given speed
  public CommandBase setMotor(DoubleSupplier speed){
    return runOnce(() -> {testMotor.set(speed.getAsDouble()); });
  }

  double fixedMod(double a, double b){
    double bad = a % b;
    return bad + (bad < 0? b: 0);
}
  @Override
  public void periodic() {
    offset += encoder.getVelocity()/5500;
    if(beamBreak.get()){
      for(var i = 0; i < led_count; i++){
        buffer.setRGB(i,0,0,(int)(64 * fixedMod(i + offset,12)/12));
      }
    }
    else{
      for(var i = 0; i < led_count; i++){
        buffer.setRGB(i,(int)(64 * fixedMod(i + offset,12)/12),0,0);
      }
    }
    
    
    leds.setData(buffer);
    
    
  }
}
