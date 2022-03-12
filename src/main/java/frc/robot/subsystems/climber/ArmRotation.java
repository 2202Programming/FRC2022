// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.ClimbSettings;

public class ArmRotation {
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

    // calibration
    private boolean calibrate = false;
    private NetworkTableEntry nte_calibrate;

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

        nte_calibrate = this.network_table.getEntry("Calibrate");
        nte_calibrate.setBoolean(calibrate);

        nte_backward_limit = network_table.getEntry("Backward Limit");
        nte_backward_limit.setBoolean(false);
        nte_forward_limit = network_table.getEntry("Forward Limit");
        nte_forward_limit.setBoolean(false);

        encoder = motor_rot.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        encoder.setInverted(inverted);
        ClimbSettings.rotatePID.copyTo(motor_rot.getPIDController(), 0);
    }

    public void startCalibration() {
        calibrate = true;
        nte_calibrate.setBoolean(calibrate);
    }

    public boolean isCalibrated() {
        return !calibrate;
    }

    public void periodic() {

        double encoder_curr_counts = encoder.getPosition() * 4096;
        

        nte_curr_pos_count.setDouble(encoder_curr_counts);
        nte_curr_pos_deg.setDouble(encoderCountsToDegrees(encoder_curr_counts));

        // current limit n stuff
        // motor_rot.setSmartCurrentLimit((int)
        // nte_curr_current_limit_amp.getDouble(5));
        nte_backward_limit.setBoolean(BackwardLimitSwitch.isPressed());
        nte_forward_limit.setBoolean(ForwardLimitSwitch.isPressed());

        // TODO: Move all calibration logic to a command
        if (calibrate) {
            if (isForwardLimitHit()) {
                setPercentOutput(0);
                calibrate = false;
                nte_calibrate.setBoolean(calibrate);
                REVLibError ret = encoder.setPosition(0);
                System.out.println("Calibration complete, setPos return " + ret + " position (deg): " + encoderCountsToDegrees(encoder.getPosition()*8192));
            } else {
                setPercentOutput(.25);
            }
        }

    }

    // TODO: update with new encoders
    public double DegreesToEncoderCounts(double degrees) {
        return ((42 * 174.9) / (360 * 20)) * degrees;
        // 42 and 20 are the tooth pullies, 174.9 is the counts per rot and 360 is the
        // degrees
        // 174.9 counter/rotation on motor
        // Motor is connected to a 20 tooth pulley that drives
        // a 42 tooth pulley
        // changes the angle of the ? by this many degrees
    }

    // TODO: fix
    public double encoderCountsToDegrees(double counts) {
        return 1 / DegreesToEncoderCounts(counts);
    }

    public void set(double degrees) {
        // TODO: use conversion factor variable?
        double counts = DegreesToEncoderCounts(degrees);
        pidController_rot.setReference(counts, CANSparkMax.ControlType.kPosition);
        nte_des_pos_deg.setDouble(degrees);
        nte_des_pos_count.setDouble(counts);
    }

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
        return encoderCountsToDegrees(encoder.getPosition());
    }
}
