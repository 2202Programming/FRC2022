// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriverPrefs;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDeployToggle;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.auto.auto_drivePath_cmd;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.subsystems.Intake_Subsystem; 
import frc.robot.ux.Dashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  static RobotContainer  rc;
  public static RobotContainer   RC() {return rc;}

  public final HID_Xbox_Subsystem driverControls;
  public final Sensors_Subsystem sensors;
  public final Intake_Subsystem intake; //New
  private final SwerveDrivetrain drivetrain;
  public final Dashboard dashboard;
  public final Limelight_Subsystem limelight;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotContainer.rc = this;

    //create our subsystems
    sensors = new Sensors_Subsystem();
    intake = new Intake_Subsystem(); //New
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone); 
    drivetrain = new SwerveDrivetrain();
    limelight = new Limelight_Subsystem();


    // set default commands
    drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverControls, limelight));

    //setup the dashboard programatically, creates any choosers, screens
    dashboard = new Dashboard(this);

    setDriverButtons();
    setAssistantButtons();
    
  }

  /**
  * Driver xbox controller button bindings
  * <ul>
  * <li> B - Toggle drive mode </li>
  * <li> A - Trajectory Follow Test </li>
  * <li> Y - Reset Pose to Zero </li>
  * <li> X - Follow path off chooser </li>
  * </ul>
  */
  void setDriverButtons(){

    //B - Toggle drive mode
    driverControls.bind(Id.Driver, XboxButton.B).whenPressed(new InstantCommand( drivetrain::driveModeCycle ));
  
    //A - Trajectory Test
    driverControls.bind(Id.Driver, XboxButton.A).whenPressed(getTrajectoryFollowTestCommand());
  
    //Y - reset Pose
    driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand( drivetrain::resetPose ));

    //X - follow path off chooser
    driverControls.bind(Id.Driver, XboxButton.X).whenPressed(new auto_drivePath_cmd(drivetrain, dashboard.getTrajectoryChooser()));

    driverControls.bind(Id.Driver, XboxButton.RB).whenPressed(new InstantCommand( limelight::toggleLED ));

  }

    /**
  * Assistant xbox controller button bindings
  * <ul>
  * <li> TBD </li>
  * </ul>
  */
  void setAssistantButtons(){

    // Y -toggle intake deploy
    // B - spin intake while held (to intake the ball)
    // A - spin intake while held (in reverse to expell the ball)

    driverControls.bind(Id.Assistant, XboxButton.Y).whenPressed(new IntakeDeployToggle());
    // IntakeCommand takes a DoubleSupplier f() which could be tied to our UX instead of const f() given here.
    driverControls.bind(Id.Assistant, XboxButton.B).whileHeld(new IntakeCommand((()-> 0.50), IntakeMode.LoadCargo) );
    // IntakeCommand motor direction
    driverControls.bind(Id.Assistant, XboxButton.A).whileHeld(new IntakeCommand((()-> 0.50), IntakeMode.ExpellCargo) );

  }

  // testing 
  void test_controls() {
    // var cmd = new MK3_AngleSpeed(driverControls, drivetrain, 0);  // FL, FR, BL, BR (0..3)
    //drivetrain.setDefaultCommand(cmd);

    //testing commands, speed in feet per sec, angle in degrees
    //driverControls.bind(Id.Driver, XboxPOV.POV_UP).whenHeld(new SwerveDriveTest(drivetrain, 1, 0));
    //driverControls.bind(Id.Driver, XboxPOV.POV_RIGHT).whenHeld(new SwerveDriveTest(drivetrain, 1, 90));
    //driverControls.bind(Id.Driver, XboxPOV.POV_DOWN).whenHeld(new SwerveDriveTest(drivetrain, 1, 180));
    //driverControls.bind(Id.Driver, XboxPOV.POV_LEFT).whenHeld(new SwerveDriveTest(drivetrain, 1, -90));
  }

  public Command getTrajectoryFollowTestCommand (){
    // An example trajectory to follow.  All units in feet.
    Rotation2d current_angle = new Rotation2d(sensors.getYaw());
    Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
        
        new Pose2d(0.0, 0.0, current_angle),
        List.of(
          // new Translation2d(0.0, 0.25),
          // new Translation2d(0.0, 0.5),
          // new Translation2d(0.0, 0.75)
        ),
        new Pose2d(0, 3.0, current_angle),
        new TrajectoryConfig(2.0, 0.5) //max velocity, max accel
        //new TrajectoryConfig(Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxAngularSpeed) //way too fast
        
      );
      
      SwerveControllerCommand swerveControllerCommand =
      new SwerveControllerCommand(
          exampleTrajectory,
          drivetrain::getPose, // Functional interface to feed supplier
          drivetrain.getKinematics(),
          // Position controllers 
          new PIDController(4.0, 0.0, 0.0),
          new PIDController(4.0, 0.0, 0.0),
          new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(.3, .3)),
            // Here, our rotation profile constraints were a max velocity
            // of 1 rotation per second and a max acceleration of 180 degrees
            // per second squared
          drivetrain::setModuleStates,
          drivetrain);

        // Reset odometry to the starting pose of the trajectory.
    drivetrain.setPose(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0)).withTimeout(10);

  }
}
