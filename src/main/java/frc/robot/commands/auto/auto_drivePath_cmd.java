// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class auto_drivePath_cmd extends CommandBase {
  /** Creates a new auto_drivePath_cmd. */

  private final SwerveDrivetrain m_robotDrive;
  SendableChooser<Trajectory> chooser;
  Trajectory path;

  public auto_drivePath_cmd(SwerveDrivetrain drive, SendableChooser<Trajectory> chooser) {
    m_robotDrive = drive;
    this.chooser = chooser;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // grab the trajectory determined by the AutoPath
    path = chooser.getSelected();
    if (path != null) {
      // Reset odometry to the starting pose of the trajectory.
      m_robotDrive.setPose(path.getInitialPose());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command runcommand = getPathCommand();
    runcommand.schedule();
  }

  public Command getPathCommand() {
    if (path == null) {
      return new InstantCommand();  // no path selected
    }
      
      SwerveControllerCommand swerveControllerCommand =
      new SwerveControllerCommand(
          path,
          m_robotDrive::getPose, // Functional interface to feed supplier
          m_robotDrive.getKinematics(),
          // Position controllers 
          new PIDController(4.0, 0.0, 0.0),
          new PIDController(4.0, 0.0, 0.0),
          new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(.3, .3)),
            // Here, our rotation profile constraints were a max velocity
            // of 1 rotation per second and a max acceleration of 180 degrees
            // per second squared
            m_robotDrive::drive,
            m_robotDrive
            );

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.setPose(path.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.stop()).withTimeout(20);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
