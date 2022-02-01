// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public interface SetsPercentOutput extends Subsystem {
    public void setPercentOutput(double speed);
}
