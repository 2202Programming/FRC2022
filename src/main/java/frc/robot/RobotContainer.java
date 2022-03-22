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
import frc.robot.commands.Shoot.VelShootCommand;
import frc.robot.commands.auto.auto_cmd;
import frc.robot.commands.climber.MidClimb;
import frc.robot.commands.climber.MoveArmsTo;
import frc.robot.commands.climber.PitAlignClimber;
import frc.robot.commands.climber.TraverseClimb;
import frc.robot.commands.swerve.DriveController;
import frc.robot.commands.swerve.LimelightDriveCmd;
import frc.robot.commands.test.ClimberTestRotRate;
import frc.robot.commands.test.ClimberTestVelocity;
import frc.robot.commands.test.LightGateTest;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Positioner_Subsystem;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.XboxAxis;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.hid.XboxPOV;
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
  public Climber climber = null;
  public final Limelight_Subsystem limelight;
  public final Positioner_Subsystem positioner;

  public static String auto_path_name = "NONE";

  public static DriveController m_driveController = null;

  // modifiable commands
  // DriveCmd swd;
  LimelightDriveCmd swd;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this;

    // these can get created on any hardware setup
    sensors = new Sensors_Subsystem();
    dashboard = new Dashboard(rc);
    limelight = new Limelight_Subsystem();
    positioner = new Positioner_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone);

    // These are hardware specific
    if (Constants.HAS_DRIVETRAIN)
      drivetrain = new SwerveDrivetrain();
    if (Constants.HAS_SHOOTER)
      shooter = new Shooter_Subsystem();
    if (Constants.HAS_MAGAZINE)
      magazine = new Magazine_Subsystem();
    if (Constants.HAS_INTAKE) {
      intake = new Intake_Subsystem();
      ///intake.setDefaultCommand(new LightGateTest());
    }
    if (Constants.HAS_CLIMBER)
      climber = new Climber();

    // set default commands

    if (Constants.HAS_DRIVETRAIN && Constants.HAS_SHOOTER && Constants.HAS_MAGAZINE) {
      // swd = new DriveCmd(drivetrain, driverControls);
      // swd = new LimelightDriveCmd(drivetrain, driverControls, limelight);
      m_driveController = new DriveController();
      // drivetrain.setDefaultCommand(m_driveController);
    }

    // //setup the dashboard programatically, creates any choosers, screens
    // dashboard = new Dashboard(this);

    setDriverButtons();
    setAssistantButtons();
    
    // Sideboard 
    if (Constants.HAS_CLIMBER) {
      // warning - PitAlign command use Driver's DPAD, RB and, LB. DPL-can we run this in TEST mode?
      driverControls.bind(Id.SwitchBoard, SBButton.Sw21).whileHeld(new PitAlignClimber(driverControls, Id.Driver, climber, 2.0, 5.0)); //[in/s] [deg/s]
      driverControls.bind(Id.SwitchBoard, SBButton.Sw22).whenPressed(new MidClimb(climber));
      driverControls.bind(Id.SwitchBoard, SBButton.Sw23).whenPressed(new TraverseClimb(climber));
      driverControls.bind(Id.SwitchBoard, SBButton.Sw24).whenPressed(new MoveArmsTo(climber, "To zero", 0, 0, true, true));
      driverControls.bind(Id.SwitchBoard, SBButton.Sw25).whileHeld(new ClimberTestVelocity(climber, 6, 12, 24)); //use pit-zero to start
            //new ClimberTestRotRate(climber, 15, -30, 60));
            //new ClimberTestRotRatePercent(climber, 0.5, -20, 40));
            //new ClimberTestRotOscillation(climber));
    }
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
      //driverControls.bind(Id.Driver, XboxButton.X).whenPressed(new InstantCommand(drivetrain::resetAnglePose));
      driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand(() -> { drivetrain.setPose(Autonomous.startPose3); }));
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenPressed(m_driveController::setRobotCentric);
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenReleased(m_driveController::setFieldCentric);   
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenPressed(m_driveController::turnOnShootingMode);
      driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenReleased(m_driveController::turnOffShootingMode);
    }

    // RB limelight toggle
    driverControls.bind(Id.Driver, XboxButton.X).whenPressed(new InstantCommand(limelight::toggleLED));


  }

  // /**
  // * Assistant xbox controller button bindings
  // * <ul>
  // * <li> TBD </li>
  // * </ul>
  // */
  void setAssistantButtons() {
    // LB - toggle intake deploy
    // B - spin intake while held (to intake the ball)
    // A - spin intake while held (in reverse to expell the ball)
    // RT - spin shooter and index while held
    driverControls.bind(Id.SwitchBoard, SBButton.Sw13).whenActive(new ResetPosition(Autonomous.startPose3));

    if (Constants.HAS_INTAKE) {
      driverControls.bind(Id.Assistant, XboxButton.LB).whenPressed(new MoveIntake(DeployMode.Toggle));
      driverControls.bind(Id.Assistant, XboxButton.A).whileHeld(new IntakeCommand((() -> 0.47), () -> 0.20, IntakeMode.LoadCargo));
      driverControls.bind(Id.Assistant, XboxButton.B).whileHeld(new IntakeCommand((() -> 0.35), () -> 0.20, IntakeMode.ExpellCargo));
    }

    if (Constants.HAS_MAGAZINE) {
      // Positioner binds :)
      driverControls.bind(Id.Assistant, XboxButton.RB).whenPressed(new MovePositioner(PositionerMode.Toggle));

      // MagazineCommand to intake or expell ball
      driverControls.bind(Id.Assistant, XboxButton.X).whileHeld(new MagazineCommand((() -> 1.0), MagazineMode.LoadCargo));
      driverControls.bind(Id.Assistant, XboxButton.Y).whileHeld(new MagazineCommand((() -> 1.0), MagazineMode.ExpellCargo));
    }

    if (Constants.HAS_SHOOTER) {
      driverControls.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whileHeld(new VelShootCommand(Shooter.DefaultSettings, 20)); 
      driverControls.bind(Id.Assistant, XboxPOV.POV_LEFT).whileHeld(new VelShootCommand(Shooter.shortVelocity, false));
      driverControls.bind(Id.Assistant, XboxPOV.POV_DOWN).whileHeld(new VelShootCommand(Shooter.mediumVelocity, false));
      driverControls.bind(Id.Assistant, XboxPOV.POV_RIGHT).whileHeld(new VelShootCommand(Shooter.longVelocity, false));
    }
  }

  public Command getAutonomousCommand() {
    return new auto_cmd();
  }

}
