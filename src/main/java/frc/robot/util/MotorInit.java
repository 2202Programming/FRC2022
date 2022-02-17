// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Dictionary;
import java.util.Hashtable;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class MotorLogging extends SubsystemBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("motors");
    private Hashtable<String, LoggingAdapter> motors = new  Hashtable<String, LoggingAdapter> ();
    private int ticks;
    
    
    public void add(String name, LoggingAdapter adpater) {
        adpater.init(table.getSubTable(name));
        motors.put(name, adpater);
    }

    @Override
    public void periodic() {
        if (ticks ++ > 20) {
            ticks = 0;

            for (var i : motors.values()) {
                i.log();
            }
        }
    }
}

interface LoggingAdapter {
    void init(NetworkTable nt);
    void log();
}

class CANSparkMaxLoggingAdapter implements LoggingAdapter {
    private CANSparkMax motor;
    private NetworkTableEntry appliedVoltage;
    // private NetworkTableEntry encoder;
    private NetworkTableEntry busVoltage;
    private NetworkTableEntry speed;
    private NetworkTableEntry temperature;
    
    public CANSparkMaxLoggingAdapter(CANSparkMax motor) {
        this.motor = motor;
    }

    @Override
    public void init(NetworkTable nt) {
        appliedVoltage = nt.getEntry("applied-voltage");
        busVoltage = nt.getEntry("bus-voltage");
        speed = nt.getEntry("speed");
        temperature = nt.getEntry("temperature");

        nt.getEntry("firmware").setString(motor.getFirmwareString());
    }

    @Override
    public void log() {
        appliedVoltage.setDouble(motor.getAppliedOutput());
        busVoltage.setDouble(motor.getBusVoltage());
        speed.setDouble(motor.get());
        temperature.setDouble(motor.getMotorTemperature());
    }
}

/** Add your docs here. */
public class MotorInit {


    private static MotorLogging instance = new MotorLogging();


    public static CANSparkMax SparkMax(String name, int canId, CANSparkMax.MotorType mt) {
        var motor = new CANSparkMax(canId, mt);

        var sb = new StringBuilder();
        sb.append("motor configured name=");
        sb.append(name);
        sb.append(" can=");
        sb.append(canId);
        sb.append(" faults=");
        sb.append(motor.clearFaults().toString());
        sb.append(" firmware=");
        sb.append(motor.getFirmwareString());
        
        // final log
        System.out.println(sb.toString());

        if (instance != null) {
            instance.add(name, new CANSparkMaxLoggingAdapter(motor));
        }

        return motor;
    }


}
