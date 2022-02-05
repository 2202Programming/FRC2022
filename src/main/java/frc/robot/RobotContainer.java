// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriverPrefs;
import frc.robot.commands.swerve.DriveCmd;
import frc.robot.commands.swerve.LimelightDriveCmd;
import frc.robot.commands.Shooter_MagazineCommand;
import frc.robot.commands.auto.auto_drivePath_cmd;
import frc.robot.commands.auto.auto_pathPlanner_cmd;
import frc.robot.subsystems.Intake_Subsystem;
//import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls.Id;
//import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.ux.Dashboard;
import frc.robot.commands.test.TestShoot;
import frc.robot.commands.test.dumbshooter;
import frc.robot.commands.test.SwerveDriveTest;
//test commands
import frc.robot.commands.test.getTrajectoryFollowTest;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  static RobotContainer rc;

  public static RobotContainer RC() {
    return rc;
  }

  public final Dashboard dashboard;
  public Shooter_Subsystem shooter = null;
  public final HID_Xbox_Subsystem driverControls;
  public Sensors_Subsystem sensors = null;
  public Intake_Subsystem intake = null; 
  private SwerveDrivetrain drivetrain = null;
  public Magazine_Subsystem magazine = null;
  public final Limelight_Subsystem limelight;

  //modifiable commands
  //DriveCmd swd;
  LimelightDriveCmd swd;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this;

    // create our subsystems
    
    //these can get created on any hardware setup
    sensors = new Sensors_Subsystem();
    shooter = new Shooter_Subsystem();
    dashboard = new Dashboard(rc);
    limelight = new Limelight_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone);
   
    //These are hardware specific
    if (Constants.HAS_DRIVETRAIN) drivetrain = new SwerveDrivetrain();
    if (Constants.HAS_SHOOTER) shooter = new Shooter_Subsystem();
    if (Constants.HAS_MAGAZINE) magazine = new Magazine_Subsystem();
    if (Constants.HAS_INTAKE) intake = new Intake_Subsystem();


    // set default commands
    //drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverControls, limelight));
    if (Constants.HAS_SHOOTER) shooter.setDefaultCommand(new TestShoot(shooter));
    
    if (Constants.HAS_DRIVETRAIN) {
      //swd = new DriveCmd(drivetrain, driverControls);
      swd = new LimelightDriveCmd(drivetrain, driverControls, limelight);
      drivetrain.setDefaultCommand(swd);
    }
    
    // //setup the dashboard programatically, creates any choosers, screens
    // dashboard = new Dashboard(this);

    setDriverButtons();
    setAssistantButtons();
  }

  /**
   * Driver xbox controller button bindings
   * <ul>
   * <li>B - Toggle drive mode</li>
   * <li>A - Trajectory Follow Test</li>
   * <li>Y - Reset Pose to Zero</li>
   * <li>X - Follow path off chooser</li>
   * </ul>
   */
  void setDriverButtons() {
    // B - Toggle drive mode
    if (Constants.HAS_DRIVETRAIN) {
      driverControls.bind(Id.Driver, XboxButton.B).whenPressed(new InstantCommand(swd::cycleDriveMode));
    }
    // A - Trajectory Test
    if (Constants.HAS_DRIVETRAIN) 
      driverControls.bind(Id.Driver, XboxButton.A).whenPressed(new getTrajectoryFollowTest(sensors,drivetrain));
        //.whenPressed(new SwerveDriveTest(drivetrain, 1, 0).withTimeout(8));

    // Y - reset Pose
    if (Constants.HAS_DRIVETRAIN) driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand(drivetrain::resetPose));

    // X - follow path off chooser
    if (Constants.HAS_DRIVETRAIN) {
      driverControls.bind(Id.Driver, XboxButton.X)
          //.whenPressed(new auto_drivePath_cmd(drivetrain, dashboard.getTrajectoryChooser()));
          .whenPressed(new auto_pathPlanner_cmd(drivetrain, "CenterFace"));
      driverControls.bind(Id.Driver, XboxButton.LB)
          //.whenPressed(new auto_drivePath_cmd(drivetrain, dashboard.getTrajectoryChooser()));
          .whenPressed(new auto_pathPlanner_cmd(drivetrain, "Straight5"));

    }

    //RB limelight toggle
    driverControls.bind(Id.Driver, XboxButton.RB).whenPressed(new InstantCommand( limelight::toggleLED ));
  }

  // /**
  // * Assistant xbox controller button bindings
  // * <ul>
  // * <li> TBD </li>
  // * </ul>
  // */
  void setAssistantButtons() {

  }

  
}
