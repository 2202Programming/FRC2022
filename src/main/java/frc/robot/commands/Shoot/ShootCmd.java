// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class ShootCmd extends CommandBase {
  /** Creates a new ShootCmd. */

  SwerveDrivetrain m_drivetrain;

  public ShootCmd(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setShootingMode(true);
    m_drivetrain.setDriveModeString("Shooting mode");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setShootingMode(false);
    m_drivetrain.setDriveModeString("NONE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
