// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.DriverPrefs;
import frc.robot.commands.climber.*;
import frc.robot.commands.test.ClimberTestRotRate;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls.Id;
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
  // public final Sensors_Subsystem sensors;
  // private final MecanumDriveIfx drivetrain;
  public final Dashboard dashboard;
  private final Climber climber;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotContainer.rc = this;

    //create our subsystems
    // sensors = new Sensors_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone); 
    // drivetrain = new MecanumDrivetrain();

    // set default commands
    // drivetrain.setDefaultCommand(new MecanumDriveCmd(drivetrain, driverControls));

    //setup the dashboard programatically, creates any choosers, screens
    dashboard = new Dashboard(this);

    climber = new Climber();
    //climber.setDefaultCommand(
      //new SequentialCommandGroup(
      // new CalibrateClimber(climber),
      //new climberTest(climber)
    //));

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
    driverControls.bind(Id.Driver, XboxButton.START).whileHeld(new ClimberTestRotRate(climber, 18, -30, 30));
    driverControls.bind(Id.Driver, XboxButton.X).whenPressed(new MidClimb(climber));
    //driverControls.bind(Id.Driver, XboxButton.B).whileHeld(new SetArmRotation(driverControls, climber, -0.5));
    //driverControls.bind(Id.Driver, XboxButton.Y).whileHeld(new SetArmExtension(driverControls, climber, 0.2));
    //driverControls.bind(Id.Driver, XboxButton.A).whileHeld(new SetArmExtension(driverControls, climber, -0.2));
  
    //this shouold be on the sideboard
    driverControls.bind(Id.Driver, XboxButton.B ).toggleWhenPressed(new PitAlignClimber(driverControls, Id.Driver, climber, 2.0, 10.0 ));

    //B - Toggle drive mod
    //driverControls.bind(Id.Driver, XboxButton.B).whenPressed(new InstantCommand( drivetrain::driveModeCycle ));
  
    //A - Trajectory Test
    //driverControls.bind(Id.Driver, XboxButton.A).whenPressed(getTrajectoryFollowTestCommand());
  
    //Y - reset Pose
    //driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand( drivetrain::resetPose ));

    //X - follow path off chooser
    //driverControls.bind(Id.Driver, XboxButton.X).whenPressed(new auto_drivePath_cmd(drivetrain, dashboard.getTrajectoryChooser()));
  }

    /**
  * Assistant xbox controller button bindings
  * <ul>
  * <li> TBD </li>
  * </ul>
  */
  void setAssistantButtons(){

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

  
}
