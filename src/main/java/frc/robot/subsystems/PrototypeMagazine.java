// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.SetsPercentOutput;

public class PrototypeMagazine extends SubsystemBase implements SetsPercentOutput {
  private WPI_TalonSRX motor;

  /** Creates a new PrototypeMagazine. */
  public PrototypeMagazine(int motorCanID) {
    motor = new WPI_TalonSRX(motorCanID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setPercentOutput(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }
}
