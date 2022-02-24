// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import javax.tools.StandardJavaFileManager.PathFactory;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class auto_pathPlanner_cmd extends CommandBase {
  /** Creates a new auto_pathPlanner_cmd. */

  private final SwerveDrivetrain m_robotDrive;

  // Since a PathPlannerTrajectory extends the WPILib Trajectory, it can be referenced as one
  // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
  PathPlannerTrajectory path;
  String pathname;
  Command runcommand;

  public auto_pathPlanner_cmd(SwerveDrivetrain drive, String pathname) {
    m_robotDrive = drive;
    this.pathname = pathname;
        // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    path = PathPlanner.loadPath(pathname, 3, 3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // grab the trajectory determined by the AutoPath
    if (path != null) {
      // Reset odometry to the starting pose of the trajectory.
      //IMPORTANT: Pathplanner heading of first point is the assumed starting heading of your bot
      //If first point has a non-zero heading, the gryo will get offset with this setPose
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
      System.out.println("No path");
      return new InstantCommand();  // no path selected
    }
      
      PIDController xController = new PIDController(4.0, 0.0, 0.0);
      PIDController yController = new PIDController(4.0, 0.0, 0.0);
      ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(3, 3));
      //Units are radians for thetaController; PPSwerveController is using radians internally.
      thetaController.enableContinuousInput(-Math.PI, Math.PI); //prevent piroutte paths over continuity

      PPSwerveControllerCommand swerveControllerCommand =
      new PPSwerveControllerCommand(
          path,
          m_robotDrive::getPose, // Functional interface to feed supplier
          m_robotDrive.getKinematics(),
          // Position controllers 
          xController,
          yController,
          thetaController,
          m_robotDrive::drive,
          m_robotDrive
      );

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.setPose(path.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.stop()).withTimeout(20);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  public static Command PathFactory(SwerveDrivetrain m_robotDrive, String pathname){
    var path = PathPlanner.loadPath(pathname, 3, 3);

    if (path == null) {
      return new InstantCommand();  // no path selected
    }
      
      PIDController xController = new PIDController(4.0, 0.0, 0.0);
      PIDController yController = new PIDController(4.0, 0.0, 0.0);
      ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(3, 3));
      //Units are radians for thetaController; PPSwerveController is using radians internally.
      thetaController.enableContinuousInput(-Math.PI, Math.PI); //prevent piroutte paths over continuity

      PPSwerveControllerCommand swerveControllerCommand =
      new PPSwerveControllerCommand(
          path,
          m_robotDrive::getPose, // Functional interface to feed supplier
          m_robotDrive.getKinematics(),
          // Position controllers 
          xController,
          yController,
          thetaController,
          m_robotDrive::drive,
          m_robotDrive
      );

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.setPose(path.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.stop()).withTimeout(20);
  }
}
