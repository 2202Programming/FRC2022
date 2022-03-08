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
  // motors n stuff
  private CANSparkMax motor_rot;
  private SparkMaxPIDController pidController_rot;
  private SparkMaxLimitSwitch ForwardLimitSwitch;
  private SparkMaxLimitSwitch ReverseLimitSwitch;

  private RelativeEncoder encoder;

  // nts
  private NetworkTable network_table;
  private NetworkTableEntry nte_curr_pos_deg;
  private NetworkTableEntry nte_curr_pos_count;
  private NetworkTableEntry nte_des_pos_deg;
  private NetworkTableEntry nte_des_pos_count;

  private NetworkTableEntry nte_lower_limt;
  private NetworkTableEntry nte_upper_limt;

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
      ReverseLimitSwitch = motor_rot.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
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

      nte_lower_limt = network_table.getEntry("Lower Limit");
      nte_lower_limt.setBoolean(false);
      nte_upper_limt = network_table.getEntry("Upper Limit");
      nte_upper_limt.setBoolean(false);

      encoder = motor_rot.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kQuadrature, 1);
      ClimbSettings.rotatePID.copyTo(motor_rot.getPIDController(), 0);
  }

  public void periodic() {
      
      double motor_curr_deg = encoder.getPosition();

      nte_curr_pos_count.setDouble(motor_curr_deg);
      nte_curr_pos_deg.setDouble(DegreesToEncoderCounts(motor_curr_deg));
      
      // current limit n stuff
      // motor_rot.setSmartCurrentLimit((int) nte_curr_current_limit_amp.getDouble(5));
      nte_lower_limt.setBoolean(ReverseLimitSwitch.isPressed());
      nte_upper_limt.setBoolean(ForwardLimitSwitch.isPressed());

      if (calibrate) {
          if (isLowerLimitHit()) {
              setPercentOutput(0);
              calibrate = false;
              nte_calibrate.setBoolean(calibrate);
              encoder.setPosition(0);
          } else {
              setPercentOutput(.5);
          }
      }

  }

  public double DegreesToEncoderCounts(double degrees) {
      return (42 * 174.9 * degrees) / (360 * 20);
      // 42 and 20 are the tooth pullies, 174.9 is the counts per rot and 360 is the
      // degrees
      // 174.9 counter/rotation on motor
      // Motor is connected to a 20 tooth pulley that drives
      // a 42 tooth pulley
      // changes the angle of the ? by this many degrees
  }

  public double encoderCountsToDegrees(double counts) {
      return (counts * (360 * 20)) / (42 * 174.9);
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

  public boolean isLowerLimitHit() {
      return ForwardLimitSwitch.isPressed();
  }

  public double getRotationDegrees() {
      return encoderCountsToDegrees(encoder.getPosition());
  }
}
