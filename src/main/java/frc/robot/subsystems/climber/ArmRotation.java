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
    final double kCounts2Degrees = 360 / (kGR*8192);   // 360[deg]  / gr* encoder counts/rot
    final double kDegrees2Counts = 1.0 / kCounts2Degrees;

    // motors n stuff
    private CANSparkMax motor_rot;
    private SparkMaxPIDController pidController_rot;
    private SparkMaxLimitSwitch ForwardLimitSwitch;
    private SparkMaxLimitSwitch BackwardLimitSwitch;

    private RelativeEncoder encoder;

    // nts
    private NetworkTable network_table;
    private NetworkTableEntry nte_curr_pos_deg;
    private NetworkTableEntry nte_curr_pos_count;
    private NetworkTableEntry nte_des_pos_deg;
    private NetworkTableEntry nte_des_pos_count;

    private NetworkTableEntry nte_backward_limit;
    private NetworkTableEntry nte_forward_limit;

    public ArmRotation(NetworkTable table, CANSparkMax motor_rot, boolean inverted) {

        // motors
        this.motor_rot = motor_rot;
        this.motor_rot.restoreFactoryDefaults();
        this.motor_rot.clearFaults();
        this.motor_rot.setInverted(inverted);
        setPercentOutput(0);

        ForwardLimitSwitch = motor_rot.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        BackwardLimitSwitch = motor_rot.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        this.pidController_rot = motor_rot.getPIDController();

        // NTs
        this.network_table = table;
        nte_curr_pos_deg = network_table.getEntry("Current Arm Rotation (degrees)");
        nte_curr_pos_deg.setDouble(0);
        nte_curr_pos_count = network_table.getEntry("Current Arm Rotation (counts)");
        nte_curr_pos_count.setDouble(0);
        nte_des_pos_deg = network_table.getEntry("Desired Arm Rotation (degrees)");
        nte_des_pos_deg.setDouble(0);
        nte_des_pos_count = network_table.getEntry("Desired Arm Rotation (counts)");
        nte_des_pos_count.setDouble(0);

        nte_backward_limit = network_table.getEntry("Backward Limit");
        nte_backward_limit.setBoolean(false);
        nte_forward_limit = network_table.getEntry("Forward Limit");
        nte_forward_limit.setBoolean(false);

        encoder = motor_rot.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        encoder.setInverted(inverted);
        ClimbSettings.rotatePID.copyTo(motor_rot.getPIDController(), 0);
        //TODO:    motor_ext.setSmartCurrentLimit((int) nte_motor_amp_limit.getDouble(5)); // pick a good number for current limit
    }

    public void periodic() {
        double encoder_curr_counts = encoder.getPosition();
        
        nte_curr_pos_count.setDouble(encoder_curr_counts);
        nte_curr_pos_deg.setDouble(kCounts2Degrees*encoder_curr_counts);
        nte_backward_limit.setBoolean(BackwardLimitSwitch.isPressed());
        nte_forward_limit.setBoolean(ForwardLimitSwitch.isPressed());
    }

    public void setEncoderPos(double pos) {
        encoder.setPosition(pos);
    }

    public void set(double degrees) {
        double counts = kDegrees2Counts*degrees;
        pidController_rot.setReference(counts, CANSparkMax.ControlType.kPosition);
        nte_des_pos_deg.setDouble(degrees);
        nte_des_pos_count.setDouble(counts);
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
        return kCounts2Degrees*encoder.getPosition();
    }
}
