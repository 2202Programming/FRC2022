
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.robot.util.PIDFController;

public class FlywheelConfig {
    public PIDFController pid;
    public double maxOpenLoopRPM; 
    public double gearRatio;           // account for gearbox reduction to flywheel
    public boolean sensorPhase;
    public boolean inverted;

    //flywheel radius measured in feet
    public double flywheelRadius;
    public double FWrpe2MU;            // FlywheelRPM to Motor-Units  (includes gearing)
  }