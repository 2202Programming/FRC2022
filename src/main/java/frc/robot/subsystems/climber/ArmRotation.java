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

public class ArmRotation {
    // encoder is 1:1 with motor and drives small-gear (12 tooth)
    // small-gear drives 26 tooth large gear
    final double kGR = 26.0 / 12.0;   // motor rotations to arm rot[deg]
    final double kCounts2Degrees = 360 / kGR;   // 360[deg]  / gr* encoder counts/rot

    final double kAff;
    
    // vel and pos have their own pid values in constants and on the controller
    final int vel_pid = 0;
    final int pos_pid = 1;

    // motors n stuff
    private CANSparkMax motor_rot;
    private SparkMaxPIDController pidController;
    private SparkMaxLimitSwitch ForwardLimitSwitch;
    private SparkMaxLimitSwitch BackwardLimitSwitch;

    private RelativeEncoder encoder;

    // nts
    private NetworkTable network_table;
    private NetworkTableEntry nte_curr_pos_deg;
    private NetworkTableEntry nte_curr_vel_deg;
    private NetworkTableEntry nte_des_pos_deg;
    private NetworkTableEntry nte_duty_cycle;
    private NetworkTableEntry nte_amps;

    private NetworkTableEntry nte_backward_limit;
    private NetworkTableEntry nte_forward_limit;

    public ArmRotation(NetworkTable table, CANSparkMax motor_rot, boolean inverted, double kAff) {
        this.kAff = kAff;
        // motors
        this.motor_rot = motor_rot;
        this.motor_rot.restoreFactoryDefaults();
        this.motor_rot.clearFaults();
        this.motor_rot.setInverted(inverted);

        ForwardLimitSwitch = motor_rot.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        BackwardLimitSwitch = motor_rot.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        pidController = motor_rot.getPIDController();

        // NTs
        this.network_table = table;
        nte_curr_pos_deg = network_table.getEntry("Arm Rotation");
        nte_curr_vel_deg = network_table.getEntry("Arm RotRate");
        nte_curr_pos_deg.setDouble(0);
        nte_des_pos_deg = network_table.getEntry("PoS-CL Target Rotation");  //not used
        nte_des_pos_deg.setDouble(99.99);
        nte_duty_cycle = network_table.getEntry("Duty Cycle");
        nte_amps = network_table.getEntry("Output Amps");
        nte_backward_limit = network_table.getEntry("Backward Limit");
        nte_backward_limit.setBoolean(false);
        nte_forward_limit = network_table.getEntry("Forward Limit");
        nte_forward_limit.setBoolean(false);

        // Encoder setup - use native units everywhere.
        encoder = motor_rot.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        encoder.setInverted(inverted);
        encoder.setPositionConversionFactor(kCounts2Degrees);
        encoder.setVelocityConversionFactor(kCounts2Degrees/60.0);

        //Set PIDF values on the controller for vel and pos closed loop
        ClimbSettings.rotatePID_pos.copyTo(pidController, pos_pid);
        ClimbSettings.rotatePID_vel.copyTo(pidController, vel_pid);
    }

    public void periodic() {
        nte_curr_pos_deg.setDouble(getRotationDegrees());
        nte_curr_vel_deg.setDouble(-encoder.getVelocity());
        nte_backward_limit.setBoolean(BackwardLimitSwitch.isPressed());
        nte_forward_limit.setBoolean(ForwardLimitSwitch.isPressed());
        nte_duty_cycle.setDouble(motor_rot.getAppliedOutput());
        nte_amps.setDouble(motor_rot.getOutputCurrent());
    }

    public void setEncoderPos(double pos) {
        encoder.setPosition(pos);
    }

    public void set(double degrees) {
        pidController.setReference(degrees, CANSparkMax.ControlType.kPosition, pos_pid);
        nte_des_pos_deg.setDouble(degrees);
        System.out.println("warning closed loop Rot position given - not expected");
    }

    /**
     * 
     * @param rate [deg/s]
     */
    public void setRotRate(double rate) {
        //account for the sign convention here 
        double arbFF = kAff * Math.signum(-rate);
        pidController.setReference(-rate, CANSparkMax.ControlType.kVelocity, vel_pid, arbFF);
        
    }

    public void resetPID() {
        pidController.setIAccum(0.0);
    }

    // calibration and testing only
    public void setPercentOutput(double percentOutput) {
        motor_rot.set(percentOutput);
    }

    public boolean isBackwardLimitHit() {
        return BackwardLimitSwitch.isPressed();
    }

    public boolean isForwardLimitHit() {
        return ForwardLimitSwitch.isPressed();
    }

    public double getRotationDegrees() {
        // account for sign convention here
        return -encoder.getPosition();
    }
}
