// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ifx.MecanumDriveIfx;
import frc.robot.Constants.PWM;
import edu.wpi.first.wpilibj.drive.MecanumDrive;


public class MecanumDrivetrain extends SubsystemBase implements MecanumDriveIfx{

  // Create controllers
  Spark FL = new Spark(PWM.mech_FL);
  Spark FR = new Spark(PWM.mech_FR);
  Spark BL = new Spark(PWM.mech_BL);
  Spark BR = new Spark(PWM.mech_BR);

  MecanumDrive drive;

    /** Creates a new Mechanum. */
  public MecanumDrivetrain() {
    drive = new MecanumDrive(FL,BL,FR,BR);
  }

  @Override
  public void drive_normalized(double xSpeed, double ySpeed, double rotation) {
    drive.driveCartesian(xSpeed, ySpeed, rotation);
    
  }
}
