// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    final double kGR = 36;   // neo rotations : extenion gear
    final double kRotation2Inches = 5.5/kGR;             // 5.5 inch ext /gr [Neo Rotation:output-gear]
    
    // vel and pos have their own pid values in constants and on the controller
    final int vel_pid = 0;
    final int pos_pid = 1;

    private NetworkTable network_table;
    //motor controls
    private SparkMaxPIDController pidController;
    private SparkMaxLimitSwitch ForwardLimitSwitch;
    private SparkMaxLimitSwitch ReverseLimitSwitch;
    private CANSparkMax motor_ext;
    private RelativeEncoder encoder;

    private NetworkTableEntry nte_velocity;
    private NetworkTableEntry nte_position;
    private NetworkTableEntry nte_des_pos_in;
    private NetworkTableEntry nte_motor_amp_limit;
    private NetworkTableEntry nte_duty_cycle;        //useful for kff in vel mode
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
        pidController= motor_ext.getPIDController();
        encoder = motor_ext.getEncoder();
        encoder.setPositionConversionFactor(kRotation2Inches);
        encoder.setVelocityConversionFactor(kRotation2Inches/60.0); //[rpm to rps]

        this.network_table = table;
        nte_velocity = table.getEntry("Arm Velocity");
        nte_position = table.getEntry("Arm Position");
        nte_des_pos_in = table.getEntry("Desired Position (inches)");
        
        nte_motor_amp_limit = this.network_table.getEntry("Motor Amp Limit");
        nte_motor_amp_limit.setDouble(40);
        nte_duty_cycle = this.network_table.getEntry("Duty Cycle");

        nte_lower_limt = this.network_table.getEntry("Lower Limit");
        nte_upper_limt = this.network_table.getEntry("Upper Limit");

        //Set PIDF values on the controller for vel and pos closed loop
        ClimbSettings.extendPID_pos.copyTo(pidController, pos_pid);
        ClimbSettings.extendPID_vel.copyTo(pidController, vel_pid);

        motor_ext.setSmartCurrentLimit((int) nte_motor_amp_limit.getDouble(5)); 
        encoder.setPosition(0.0); 
    }

    public void periodic() {
        nte_lower_limt.setBoolean(ReverseLimitSwitch.isPressed());
        nte_upper_limt.setBoolean(ForwardLimitSwitch.isPressed());
        nte_velocity.setDouble(-encoder.getVelocity());
        nte_position.setDouble(-encoder.getPosition());
        nte_duty_cycle.setDouble(motor_ext.getAppliedOutput());
    }

    public void setInches(double inches) {
        pidController.setReference(inches, CANSparkMax.ControlType.kPosition, pos_pid);
        nte_des_pos_in.setDouble(inches);
    }

    /**
     * setSpeed of arm extension
     * 
     * @param v_ext  [in/s]
     */
    public void setSpeed(double v_ext) {
        //account for the sign convention here 
        pidController.setReference(-v_ext, CANSparkMax.ControlType.kVelocity, vel_pid);
    }

    /**
     * Tell the motor controller where we think the extension is. This
     * does not move anything, just sets the reported position.
     * Used during calibration or whenever a limit is hit, we set it
     * to a known location.
     */
    public void setEncoderPos(double pos) {
        encoder.setPosition(pos);
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
        //account for sign here
        return -this.encoder.getPosition();
    }
}