// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
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

    path = PathPlanner.loadPath(pathname, 1, 1); //last two parameters are max velocity and max accelleration
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
      
    // get initial state from the trajectory
    PathPlannerState initialState = path.getInitialState();
    Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

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
        
    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
      new InstantCommand(()-> {
        m_robotDrive.setPose(startingPose);
      }),
      new autoPrint("***Running Path " + pathname),
      swerveControllerCommand,
      new InstantCommand(m_robotDrive::stop),
      new autoPrint("***Done Running Path " + pathname)
      );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  //for first path only
  public static Command PathFactory(double maxVel, double maxAcc, String pathname){
    SwerveDrivetrain m_robotDrive = RobotContainer.RC().drivetrain;
    var path = PathPlanner.loadPath(pathname, maxVel, maxAcc); //last two parameters are max velocity and max accelleration

    if (path == null) {
      return new InstantCommand();  // no path selected
    }
      
    // get initial state from the trajectory
    PathPlannerState initialState = path.getInitialState();
    Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

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


    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
      new InstantCommand(()-> {
        m_robotDrive.setPose(startingPose);
        RobotContainer.RC().sensors.setAutoStartPose(startingPose);
      }),
      new autoPrint("***Factory1: Running Path " + pathname),
      swerveControllerCommand,
      new InstantCommand(m_robotDrive::stop),
      new autoPrint("***Done Running Path " + pathname)
      );
  }

  
  //for a subsequent path
  public static Command PathFactory2(double maxVel, double maxAcc, String pathname){
    SwerveDrivetrain m_robotDrive = RobotContainer.RC().drivetrain;
    var path = PathPlanner.loadPath(pathname, maxVel, maxAcc); //last two parameters are max velocity and max accelleration

    if (path == null) {
      return new InstantCommand();  // no path selected
    }
    
    // get initial state from the trajectory
    PathPlannerState initialState = path.getInitialState();
    Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

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
        //m_robotDrive.setPose(path.getInitialPose());

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
      new InstantCommand(()-> {  
        m_robotDrive.setPose(startingPose);  //may not need path pose reset on secondary paths
      }), //should allow robot to "drive" to beginning of second path (which should just correct rotation to current gryo)
      new autoPrint("***Factory2: Running Path " + pathname),
      swerveControllerCommand,
      new InstantCommand(m_robotDrive::stop),
      new autoPrint("***Done Running Path " + pathname)
      );
  }

  //for a subsequent path
  public static Command PathFactory3(double maxVel, double maxAcc, String pathname){
    SwerveDrivetrain m_robotDrive = RobotContainer.RC().drivetrain;
    var path = PathPlanner.loadPath(pathname, maxVel, maxAcc); //last two parameters are max velocity and max accelleration

    if (path == null) {
      return new InstantCommand();  // no path selected
    }
      
    // get initial state from the trajectory
    PathPlannerState initialState = path.getInitialState();
    Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

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
        //m_robotDrive.setPose(path.getInitialPose());

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
      new InstantCommand(()-> {  
        m_robotDrive.setPose(startingPose);  //may not need path pose reset on secondary paths
      }), //should allow robot to "drive" to beginning of second path (which should just correct rotation to current gryo)
      new autoPrint("***Factory3: Running Path " + pathname),
      swerveControllerCommand,
      new InstantCommand(m_robotDrive::stop),
      new autoPrint("***Done Running Path " + pathname)
      );
  }

}
