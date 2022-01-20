// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {
  /** Creates a new Magazine2. */
  private CANSparkMax move1;
  private CANSparkMax move2;
  
  public Magazine() {
    move1 = new CANSparkMax(Constants.CAN.move1, MotorType.kBrushless);
    move2 = new CANSparkMax(Constants.CAN.move2, MotorType.kBrushless);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
