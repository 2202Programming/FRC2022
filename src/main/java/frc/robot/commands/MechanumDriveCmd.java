// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.MechanumDrive;

public class MechanumDriveCmd extends CommandBase {

  DriverControls dc;
  MechanumDrive drive;

  /** Creates a new MechanumDrive. */
  public MechanumDriveCmd(MechanumDrive drive, DriverControls dc) {
    this.dc = dc;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = dc.getVelocityX();
    var ySpeed = dc.getVelocityY();
    var rot = dc.getRotation();

    drive.drive_normalized(xSpeed, ySpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // never finishes, this can be a default command
  }
}
