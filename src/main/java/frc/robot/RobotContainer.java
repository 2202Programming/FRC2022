// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriverPrefs;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.auto.auto_drivePath_cmd;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.ux.Dashboard;
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
  public final Shooter_Subsystem shooter;
  public final HID_Xbox_Subsystem driverControls;
  public final Sensors_Subsystem sensors;
  public final Intake_Subsystem intake; 
  private final SwerveDrivetrain drivetrain;
  public final Magazine_Subsystem magazine;
  public final Limelight_Subsystem limelight;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this;

    // create our subsystems
    sensors = new Sensors_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone);
    drivetrain = new SwerveDrivetrain();
    limelight = new Limelight_Subsystem();
    shooter = new Shooter_Subsystem();
    magazine = new Magazine_Subsystem();
    intake = new Intake_Subsystem();
    dashboard = new Dashboard(rc);

    // set default commands
    drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverControls, limelight));

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
    driverControls.bind(Id.Driver, XboxButton.B).whenPressed(new InstantCommand(drivetrain::driveModeCycle));

    // A - Trajectory Test
    driverControls.bind(Id.Driver, XboxButton.A).whenPressed(new getTrajectoryFollowTest(drivetrain));

    // Y - reset Pose
    driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand(drivetrain::resetPose));

    // X - follow path off chooser
    driverControls.bind(Id.Driver, XboxButton.X)
        .whenPressed(new auto_drivePath_cmd(drivetrain, dashboard.getTrajectoryChooser()));

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
