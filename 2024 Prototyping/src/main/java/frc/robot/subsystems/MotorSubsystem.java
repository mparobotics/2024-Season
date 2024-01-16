// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

//A Subsystem to control a single NEO motor
public class MotorSubsystem extends SubsystemBase {

  //Create a SparkMAX motor controller
  private final CANSparkMax testMotorL = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax testMotorR = new CANSparkMax(12, MotorType.kBrushless);

  //Sets up the PigeonIMU
  public WPI_Pigeon2 testPigeon = new WPI_Pigeon2(17);

  private final CANdle leds = new CANdle(18);
  private final Port colorSensorPort = Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);
  
  private double led_percent = 0;
  private double bar_offset = 0;

  public double shootSpeed = 1;
  
  


  

  //get the pid controller from the motor
  //private  SparkMaxPIDController pid = testMotorL.getPIDController();

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    //set PID values
    //pid.setP(1);
    //pid.setI(0);
    //pid.setD(0);
    
    //inverts right motor
    testMotorR.setInverted(true);


    SmartDashboard.putNumber("Shooting Speed", shootSpeed);


    
  }


  double applyDeadband(double value, double range){
    return (Math.abs(value) < range)? 0: value;
  }
  void runMotors(double speed){
    double inputSpeed = applyDeadband(speed, 0.1);
    testMotorL.set(inputSpeed); 
    testMotorR.set(inputSpeed);
  }
  //A command that sets the motor to a given speed
  public CommandBase setMotor(DoubleSupplier speed){
    return runOnce(() -> {runMotors(speed.getAsDouble());});
  }


/* 
  //A command that sets the motor's setpoint to a specified angle and uses PID position control to get the motor to the target direction
  public CommandBase setPositionPID(DoubleSupplier position){
    return runOnce(() -> pid.setReference(position.getAsDouble(),ControlType.kPosition));
  }

  public CommandBase shoot(){
    double shootSpeed = SmartDashboard.getNumber("Shooting Speed", 0);
    return runOnce(() -> {testMotorL.set(shootSpeed); testMotorR.set(shootSpeed);});
  }

  */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int bar_length = 5;
    SmartDashboard.putNumber("Pigeon Roll", testPigeon.getRoll());

    Color detectedColor = colorSensor.getColor();
    leds.setLEDs((int)(255* detectedColor.red), (int)(255*detectedColor.green), (int)(255*detectedColor.blue));
    
    
   

  }
}
