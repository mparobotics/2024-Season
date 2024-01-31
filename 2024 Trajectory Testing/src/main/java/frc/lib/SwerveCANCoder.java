// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;


import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** Replacement for CANcoder from Phoenix v5 deprecated */
public class SwerveCANCoder extends CANcoder{
    public final MagnetSensorConfigs config = new MagnetSensorConfigs();
    
    public SwerveCANCoder(int id){
        super(id);

        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        
        optimizeBusUtilization();
        getConfigurator().apply(config);
    }
    public double getPosition0to360(){
        return getAbsolutePosition().getValueAsDouble() * 360;
    }
}
