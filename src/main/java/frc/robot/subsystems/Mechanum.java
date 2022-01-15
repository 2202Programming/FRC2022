// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ifx.MechanumDrive;
import frc.robot.Constants;
import frc.robot.Constants.PWM;


public class Mechanum extends SubsystemBase implements MechanumDrive{

  Spark FL = new Spark(PWM.mech_FL);
  /** Creates a new Mechanum. */
  public Mechanum() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void drive_normalized(double x, double y, double rotation) {
    // TODO Auto-generated method stub
    
  }
}
