// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem;

public class MaintainMag extends CommandBase {

  private Magazine_Subsystem mMagazine;
  private DoubleSupplier topWheelSpeed;

  public MaintainMag(Magazine_Subsystem mMagazine, DoubleSupplier topWheelSpeed) {
    this.mMagazine = mMagazine;
    this.topWheelSpeed = topWheelSpeed;
    
    addRequirements(mMagazine);
  }

  /** Creates a new MaintainMag. */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mMagazine.isGate1Blocked()){
      mMagazine.driveWheelOn(topWheelSpeed.getAsDouble());
    } else {
      mMagazine.driveWheelOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mMagazine.driveWheelOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
