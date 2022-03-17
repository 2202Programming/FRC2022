// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.DriverPrefs;
import frc.robot.Constants.Shooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import static frc.robot.commands.MoveIntake.DeployMode;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MagazineCommand;
import frc.robot.commands.MagazineCommand.MagazineMode;
import frc.robot.commands.MovePositioner.PositionerMode;
import frc.robot.commands.MovePositioner;
import frc.robot.commands.ResetPosition;
import frc.robot.commands.Shoot.SuperDuperDumbShooter;
import frc.robot.commands.Shoot.VelShootCommand;
import frc.robot.commands.auto.auto_cmd_group2;
import frc.robot.commands.auto.auto_pathPlanner_cmd;
import frc.robot.commands.swerve.DriveController;
import frc.robot.commands.swerve.LimelightDriveCmd;
import frc.robot.commands.test.getTrajectoryFollowTest;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Positioner_Subsystem;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.XboxAxis;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.ux.Dashboard;

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
  public SwerveDrivetrain drivetrain = null;
  public Magazine_Subsystem magazine = null;
  public final Limelight_Subsystem limelight;
  public final Positioner_Subsystem positioner;

  public static String auto_path_name = "NONE";

  public static DriveController m_driveController = null;

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
    dashboard = new Dashboard(rc);
    limelight = new Limelight_Subsystem();
    positioner = new Positioner_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone);
   
    //These are hardware specific
    if (Constants.HAS_DRIVETRAIN) drivetrain = new SwerveDrivetrain();
    if (Constants.HAS_SHOOTER) shooter = new Shooter_Subsystem();
    if (Constants.HAS_MAGAZINE) magazine = new Magazine_Subsystem();
    if (Constants.HAS_INTAKE) intake = new Intake_Subsystem();


    // set default commands
    //drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driverControls, limelight));
    // if (Constants.HAS_SHOOTER) shooter.setDefaultCommand(new TestShoot(shooter));
    
    if (Constants.HAS_DRIVETRAIN && Constants.HAS_SHOOTER && Constants.HAS_MAGAZINE) {
      //swd = new DriveCmd(drivetrain, driverControls);
      //swd = new LimelightDriveCmd(drivetrain, driverControls, limelight);
      m_driveController = new DriveController();
      //drivetrain.setDefaultCommand(m_driveController);
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
      driverControls.bind(Id.Driver, XboxButton.B).whenPressed(m_driveController::cycleDriveMode);
    }
    // A - Trajectory Test
    if (Constants.HAS_DRIVETRAIN) 
      driverControls.bind(Id.Driver, XboxButton.A).whenPressed(new getTrajectoryFollowTest(sensors,drivetrain));
        //.whenPressed(new SwerveDriveTest(drivetrain, 1, 0).withTimeout(8));

    // Y - reset Pose
    //if (Constants.HAS_DRIVETRAIN) driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand(drivetrain::resetAnglePose));

    if (Constants.HAS_DRIVETRAIN) {
      //reset angle only
      driverControls.bind(Id.Driver, XboxButton.X).whenPressed(new InstantCommand(drivetrain::resetAnglePose));
      
      //reset angle and X,Y to start pose3
      driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand( ()-> 
      {
        drivetrain.setPose(Autonomous.startPose3);
      }));
    }
    
    //X - follow path off chooser
    // if (Constants.HAS_DRIVETRAIN) {
    //   driverControls.bind(Id.Driver, XboxButton.START)
    //       //.whenPressed(new auto_drivePath_cmd(drivetrain, dashboard.getTrajectoryChooser()));
    //       .whenPressed(auto_pathPlanner_cmd.PathFactory(drivetrain, "AutoPath4").andThen(m_driveController));
    //   // driverControls.bind(Id.Driver, XboxButton.BACK)
      //      //.whenPressed(new auto_drivePath_cmd(drivetrain, dashboard.getTrajectoryChooser()));
      //      .whenPressed(auto_pathPlanner_cmd.PathFactory(drivetrain, "Straight1").andThen(m_driveController));
    //}

    //RB limelight toggle
    driverControls.bind(Id.Driver, XboxButton.RB).whenPressed(new InstantCommand( limelight::toggleLED ));
   

    if(Constants.HAS_DRIVETRAIN){
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenPressed(m_driveController::turnOnShootingMode);
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenReleased(m_driveController::turnOffShootingMode);
      
    }
  }

  // /**
  // * Assistant xbox controller button bindings
  // * <ul>
  // * <li> TBD </li>
  // * </ul>
  // */
  void setAssistantButtons() {
    // LB - toggle intake deploy
    // B  - spin intake while held (to intake the ball)
    // A  - spin intake while held (in reverse to expell the ball)
    // RT - spin shooter and index while held
    driverControls.bind(Id.SwitchBoard, SBButton.Sw13).whenActive(new ResetPosition(Autonomous.startPose3));

    if(Constants.HAS_INTAKE) {
      driverControls.bind(Id.Assistant, XboxButton.LB).whenPressed(new MoveIntake(DeployMode.Toggle));
      // IntakeCommand takes a DoubleSupplier f() which could be tied to our UX instead of const f() given here.
      driverControls.bind(Id.Assistant, XboxButton.A).whileHeld(new IntakeCommand((()-> 0.47), ()-> 0.20,  IntakeMode.LoadCargo) );
      // IntakeCommand motor direction
      driverControls.bind(Id.Assistant, XboxButton.B).whileHeld(new IntakeCommand((()-> 0.35), ()-> 0.20, IntakeMode.ExpellCargo) );
    }

    if(Constants.HAS_MAGAZINE){
      //Positioner binds :)
      driverControls.bind(Id.Assistant, XboxButton.RB).whenPressed(new MovePositioner(PositionerMode.Toggle));

      //MagazineCommand to intake or expell ball
      driverControls.bind(Id.Assistant, XboxButton.X).whileHeld(new MagazineCommand((()-> 1.0), MagazineMode.LoadCargo) );
      driverControls.bind(Id.Assistant, XboxButton.Y).whileHeld(new MagazineCommand((()-> 1.0), MagazineMode.ExpellCargo) );
    }

    if(Constants.HAS_SHOOTER){
      driverControls.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whileHeld(new VelShootCommand(Shooter.DefaultSettings, 20)); //our smart shooting command, use this one
      driverControls.bind(Id.Assistant, XboxAxis.TRIGGER_LEFT).whileHeld(new SuperDuperDumbShooter(1)); //runs intake, magazine at default intake speeds, and shooter and full output - failsafe
    }
  }

  public Command getAutonomousCommand() {
      return new auto_cmd_group2(drivetrain, magazine, intake, driverControls); 
  }
  
}
