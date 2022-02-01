// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.interfaces.SimpleIntake;

public class PrototypeIntake extends SubsystemBase implements SimpleIntake {
  private WPI_TalonSRX motor;
  private DoubleSolenoid extension;

  /** Creates a new PrototypeIntake. */
  public PrototypeIntake(int motorCanId, int forwardChannel, int reverseChannel) {
    // create motors
    motor = new WPI_TalonSRX(motorCanId);
    extension = new DoubleSolenoid(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    
    // set to zero
    reset();
  }

  private void reset() {
    motor.clearStickyFaults();
    motor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setPercentOutput(double percent) {
    if(percent > 1.0) percent = 1.0;
    if(percent < -1.0) percent = -1.0;

    motor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void off() {
    setPercentOutput(0.0);
  }

  @Override
  public void setExtended(boolean isExtended) {
    extension.set(isExtended ? Value.kForward : Value.kOff);
  }
}
