// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//A Subsystem to control a single NEO motor
public class MotorSubsystem extends SubsystemBase {

  //Create a SparkMAX motor controller
  private final CANSparkMax testMotor = new CANSparkMax(11, MotorType.kBrushless);

  //get the pid controller from the motor
  private  SparkMaxPIDController pid = testMotor.getPIDController();

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    //set PID values
    pid.setP(1);
    pid.setI(0);
    pid.setD(0);
  }
  //A command that sets the motor to a given speed
  public CommandBase setMotor(DoubleSupplier speed){
    return runOnce(() -> {testMotor.set(speed.getAsDouble());});
  }

  //A command that sets the motor's setpoint to a specified angle and uses PID position control to get the motor to the target direction
  public CommandBase setPositionPID(DoubleSupplier position){
    return runOnce(() -> pid.setReference(position.getAsDouble(),ControlType.kPosition));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
