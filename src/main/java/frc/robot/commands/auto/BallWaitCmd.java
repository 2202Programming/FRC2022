// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Magazine_Subsystem;

public class BallWaitCmd extends CommandBase {
  /** Creates a new BallWaitCmd. */
  
  Magazine_Subsystem magazine;

  public BallWaitCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    magazine = RobotContainer.RC().magazine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return magazine.bothGatesBlocked();
  }
}
