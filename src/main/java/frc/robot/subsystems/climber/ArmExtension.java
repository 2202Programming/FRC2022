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
    final double kGR = 36;   // neo rotations to extenion
    final double kCounts2Inches = 5.5/(kGR*42.0);             // 5.5 inch ext / 42 counts [Neo Rotation]
    final double kInches2Counts = 1.0 / kCounts2Inches;

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
    private NetworkTableEntry nte_lower_limt;
    private NetworkTableEntry nte_upper_limt;

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

        nte_lower_limt = this.network_table.getEntry("Lower Limit");
        nte_upper_limt = this.network_table.getEntry("Upper Limit");
        ClimbSettings.extendPID.copyTo(motor_ext.getPIDController(), 0);

        motor_ext.setSmartCurrentLimit((int) nte_motor_amp_limit.getDouble(5));  // likey needs to be done only once

    }

    public void periodic() {
        nte_lower_limt.setBoolean(ReverseLimitSwitch.isPressed());
        nte_upper_limt.setBoolean(ForwardLimitSwitch.isPressed());
        nte_curr_pos_in.setDouble(kCounts2Inches * motor_ext.getEncoder().getPosition());
        nte_curr_pos_count.setDouble(motor_ext.getEncoder().getPosition());
        nte_amps_now.setDouble(motor_ext.getOutputCurrent());
    }

    public void setInches(double inches) {
        double count = kInches2Counts*inches;
        pidController_ext.setReference(count, CANSparkMax.ControlType.kPosition);
        nte_des_pos_in.setDouble(inches);
        nte_des_pos_count.setDouble(count);
    }

    /**
     * Tell the motor controller where we think the extension is
     * Used during calibration or whenever a limit is hit, we set it
     * to a known location.
     */
    public void setMotorPos(double pos) {
         motor_ext.getEncoder().setPosition(pos);
    }

    /**
     * used only for calibration modes and testing.
     * @param percentOutput
     */
    public void setPercentOutput(double percentOutput) {
        motor_ext.set(percentOutput);
    }

    public boolean isLowerLimitHit() {
        return ReverseLimitSwitch.isPressed();
    }

    public double getInches() {
        return this.motor_ext.getEncoder().getPosition() * kCounts2Inches;
    }
}