// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetPosition extends InstantCommand {
  private Pose2d newPose;
  private SwerveDrivetrain m_drivetrain;
  private String auto_path_name;
  
  public ResetPosition(Pose2d newPose, SwerveDrivetrain m_drivetrain, String auto_path_name) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.newPose = newPose;
    this.m_drivetrain = m_drivetrain;
    this.auto_path_name = auto_path_name;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setPose(newPose);
    RobotContainer.auto_path_name = this.auto_path_name;
  }
}