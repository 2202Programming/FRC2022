// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors_Subsystem;

public class SwingCheck extends CommandBase {
  final Sensors_Subsystem sensors;
  public enum Axis {
    Roll(0, "Roll"), Pitch(1, "Pitch"), Yaw(2, "Yaw");
    int idx;
    String str;
    private Axis(int v, String s) {
      idx = v; 
      str = s;
    }
  };

  final NetworkTableEntry nte_pos;
  final NetworkTableEntry nte_rate;

  final Axis axis;
  final double axis_min, axis_max, axis_min_rate, axis_max_rate;
  final DoubleSupplier posFunc, rateFunc;
  double pos, rate;

  /**
   * Finishes when the given axis is within the range and abs(rotation rate) is less
   * than the given rate.  
   * 
   * Consider using withTimeout() when using this function.
   * 
   * @param axis      Roll, Pitch, Yaw
   * @param min       angle [deg]
   * @param max       angle [deg]
   * @param max_rate  abs() of rate  [deg/s]
   * 
   */
  public SwingCheck(Axis axis, double min, double max, double min_rate, double max_rate ) {
    sensors = RobotContainer.RC().sensors;
    this.axis = axis;
    axis_min = min;
    axis_max = max;
    axis_min_rate = min_rate;
    axis_max_rate = max_rate;

    switch (axis) {
      case Roll:
        posFunc = sensors::getRoll;
        rateFunc = sensors::getRollRate;
        break;

      case Pitch:
        posFunc = sensors::getPitch;
        rateFunc = sensors::getPitchRate;
        break;
      case Yaw:
        posFunc = sensors::getYaw;
        rateFunc = sensors::getYawRate;
        break;
      default:
        posFunc = () -> 0.0;
        rateFunc = () -> 0.0;
    }

    NetworkTable table =   NetworkTableInstance.getDefault().getTable("/SwingCheck-"+axis.str);
    nte_rate = table.getEntry(axis.str + " rate");
    nte_pos = table.getEntry(axis.str + " pos");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pos = posFunc.getAsDouble();
    rate = rateFunc.getAsDouble();
    NTUpdate();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean pos_in_range = (pos >= axis_min) && (pos <= axis_max);
    boolean rate_in_range = (rate >= axis_min_rate) && (rate <= axis_max_rate);
    return (pos_in_range && rate_in_range);
  }

  void NTUpdate() {
    nte_pos.setDouble(pos);
    nte_rate.setDouble(rate);
  }

}
