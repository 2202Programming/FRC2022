package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climbernetwork extends CommandBase {
    private NetworkTable table;
    private NetworkTableEntry left_extender_speed, right_extender_speed, left_extender_position, right_extender_position;
    private Double last = 0.0;

    // constructor to create network table
    Climbernetwork(){
        table = NetworkTableInstance.getDefault().getTable("Climber");
        left_extender_speed = table.getEntry("Left Extender Speed");
        right_extender_speed = table.getEntry("Right Extender Speed");
        left_extender_position = table.getEntry("Left Extender Position");
        right_extender_position = table.getEntry("Right Extender Position");
    }
    @Override
    public void initialize(){
        // all intialization take care of in constructor
        // remember last value to compare to new value and change if inputed value is different
        last = 0.0;
        left_extender_speed.setDouble(last);
        right_extender_speed.setDouble(last);
        left_extender_position.setDouble(last);
        right_extender_position.setDouble(last);

    }
    @Override
    public void execute(){
        
    }
}
