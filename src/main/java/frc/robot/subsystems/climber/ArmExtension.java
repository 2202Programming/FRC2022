// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.ClimbSettings;
/**
 * Note: there are 3 limit switches --
 * 1 limit switch to check if extension is at top
 * 1 limit switch to check if extension is at bottom
 * 1 limit switch to check if rotation is vertical
 */

public class ArmExtension {
    private NetworkTable network_table;
    private SparkMaxPIDController pidController_ext;
    private SparkMaxLimitSwitch ForwardLimitSwitch;
    private SparkMaxLimitSwitch ReverseLimitSwitch;
    private CANSparkMax motor_ext;
    private NetworkTableEntry nte_curr_pos_in;
    private NetworkTableEntry nte_curr_pos_count;
    private NetworkTableEntry nte_des_pos_in;
    private NetworkTableEntry nte_des_pos_count;
    private NetworkTableEntry nte_motor_amp_limit;
    private NetworkTableEntry nte_amps_now;
    private NetworkTableEntry nte_calibrate;
    private NetworkTableEntry nte_lower_limt;
    private NetworkTableEntry nte_upper_limt;

    private final int conversion_factor = (90 / (14 + (3 / 4))); // really dumb measured math probably not accurate
    // //double count = ((25.4*36*42)/(5*24))*inches; //25.4 is mm per inch, 36 is
    // revs per 1 pev with gearbox, 42 is counts per rev and 5 is mm per 1 tooth
    // // 42 Counts per rev, but SmartMotion was showing rotations not counts
    // // Connected to a 36:1 gearbox that could change
    // // Gearbox connected to a 24 tooth pulley that is connected to a linear belt
    // // Need to ask mechs to do analysis to convert 24 tooth pulley to linear
    // distance;
    private boolean calibrate = false;

    public ArmExtension(NetworkTable table, CANSparkMax motor_ext, boolean inverted) {
        this.motor_ext = motor_ext;
        this.motor_ext.clearFaults();
        this.motor_ext.restoreFactoryDefaults();
        this.motor_ext.setInverted(inverted);
        this.motor_ext.setSmartCurrentLimit(40);

        ForwardLimitSwitch = motor_ext.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ReverseLimitSwitch = motor_ext.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        this.pidController_ext = motor_ext.getPIDController();

        this.network_table = table;
        nte_curr_pos_in = this.network_table.getEntry("Current Arm Position (inches)");
        nte_curr_pos_in.setDouble(0);
        nte_curr_pos_count = this.network_table.getEntry("Current Arm Position (count)");
        nte_curr_pos_count.setDouble(0);
        nte_des_pos_in = this.network_table.getEntry("Desired Position (inches)");
        nte_des_pos_in.setDouble(0);
        nte_des_pos_count = this.network_table.getEntry("Desired Positon (count)");
        nte_des_pos_count.setDouble(0);
        nte_motor_amp_limit = this.network_table.getEntry("Motor Amp Limit");
        nte_motor_amp_limit.setDouble(40);
        nte_amps_now = this.network_table.getEntry("Amps Now");
        nte_amps_now.setDouble(0);
        nte_calibrate = this.network_table.getEntry("Calibrate");
        nte_calibrate.setBoolean(calibrate);

        nte_lower_limt = this.network_table.getEntry("Lower Limit");
        nte_upper_limt = this.network_table.getEntry("Upper Limit");
        ClimbSettings.extendPID.copyTo(motor_ext.getPIDController(), 0);
    }

    public void startCalibration() {
        calibrate = true;
        nte_calibrate.setBoolean(calibrate);
    }

    public boolean isCalibrated() {
        return !calibrate;
    }

    public void periodic() {
        if (calibrate) {
            if (isLowerLimitHit()) {
                setPercentOutput(0);
                calibrate = false;
                nte_calibrate.setBoolean(calibrate);
                motor_ext.getEncoder().setPosition(0);
            } else {
                setPercentOutput(-.2);
            }
        }
//        setPercentOutput(0);
        nte_lower_limt.setBoolean(ReverseLimitSwitch.isPressed());
        nte_upper_limt.setBoolean(ForwardLimitSwitch.isPressed());

        nte_curr_pos_in.setDouble(motor_ext.getEncoder().getPosition() / conversion_factor);
        nte_curr_pos_count.setDouble(motor_ext.getEncoder().getPosition());
        motor_ext.setSmartCurrentLimit((int) nte_motor_amp_limit.getDouble(5));
        nte_amps_now.getDouble(motor_ext.getOutputCurrent());
    }

    public void set(double inches) {
        if (calibrate)
            return;
        double count = inches * conversion_factor;
        pidController_ext.setReference(count, CANSparkMax.ControlType.kPosition);
        // motor_ext.getPIDController().setReference(count,
        // CANSparkMax.ControlType.kPosition);
        nte_des_pos_in.setDouble(inches);
        nte_des_pos_count.setDouble(count);
    }

    public void setPercentOutput(double percentOutput) {
        motor_ext.set(percentOutput);
    }

    public boolean isLowerLimitHit() {
        return ReverseLimitSwitch.isPressed();
    }

    public double getInches() {
        return this.motor_ext.getEncoder().getPosition() * conversion_factor;
    }
}