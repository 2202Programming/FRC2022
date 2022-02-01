// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.interfaces.SetsPercentOutput;

public class TestMotorCommand extends CommandBase {
  private NetworkTable table;
  private NetworkTableEntry entry;
  private Double last = 0.0;

  private SetsPercentOutput motor;

  /** Creates a new TestMotorCommand. */
  public TestMotorCommand(String name, SetsPercentOutput motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    table = NetworkTableInstance.getDefault().getTable("Motors");
    entry = table.getEntry(name + "/percentoutput");

    this.motor = motor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    last = 0.0;
    entry.setDouble(last);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double nv = entry.getDouble(last);
    if (nv != last) { 
      last = nv;
       motor.setPercentOutput(nv); 
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
