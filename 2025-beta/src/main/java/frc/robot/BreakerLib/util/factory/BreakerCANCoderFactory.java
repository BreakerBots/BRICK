        // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.factory;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

/** Factory for producing CANcoders. */
public class BreakerCANCoderFactory {

    /**
     */
    public static CANcoder createCANCoder(int deviceID, AbsoluteSensorRangeValue absoluteSensorRange, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        
        return createCANCoder(deviceID, "rio", absoluteSensorRange, absoluteOffset, encoderDirection);
    }

    /**
     */
    public static CANcoder createCANCoder(int deviceID, String busName,
        AbsoluteSensorRangeValue absoluteSensorRange, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        CANcoder encoder = new CANcoder(deviceID, busName);
        configExistingCANCoder(encoder, absoluteSensorRange, absoluteOffset, encoderDirection);
        return encoder;
    }

    public static CANcoder createCANCoder(int deviceID, CANBus canBus, AbsoluteSensorRangeValue absoluteSensorRange, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        return createCANCoder(deviceID, canBus.getName(), absoluteSensorRange, absoluteOffset, encoderDirection);
    }

    /**
     */
    public static void configExistingCANCoder(CANcoder encoder, AbsoluteSensorRangeValue absoluteSensorRange, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        CANcoderConfiguration config =  new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = absoluteSensorRange;
        config.MagnetSensor.withMagnetOffset(absoluteOffset);
        config.MagnetSensor.SensorDirection = encoderDirection;
        encoder.getConfigurator().apply(config);    
    }
}
