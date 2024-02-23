// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



//A Subsystem to control a single NEO motor
public class MotorSubsystem extends SubsystemBase {

  
  
  private final int led_count = 120;
  private double offset = 0;
  private final AddressableLED leds = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(led_count);

  private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  //private DigitalInput beambreak = new DigitalInput(0);
  

  public NetworkTable getNoteDetector(){
    return NetworkTableInstance.getDefault().getTable("limelight-a");
  }
  public NetworkTable getAprilTagDetector(){
    return NetworkTableInstance.getDefault().getTable("limelight-b");
  }
  



  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    leds.setLength(led_count);
    leds.start();
    
    encoder.setDistancePerRotation(360);
    
  }

  public double getArmPosition(){
    return encoder.getDistance();
  }

  //A command that sets the motor to a given speed
  public Command setMotor(DoubleSupplier speed){
    return runOnce(() -> {});
  }

  double fixedMod(double a, double b){
    double bad = a % b;
    return bad + (bad < 0? b: 0);
}
  //@Override
  public void periodic() {
    //offset -= 1;
   /* for(var i = 0; i < led_count/2; i++){
      if(beambreak.get()){
        buffer.setRGB(i,0, 128,0);
      }
      else{
        buffer.setRGB(i,128, 0,0);
      }
    }*/
    
    SmartDashboard.putNumber("Arm Position", getArmPosition());
    
    leds.setData(buffer);
  }
}
