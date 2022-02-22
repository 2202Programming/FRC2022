// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.fieldCentricDrive;
import frc.robot.commands.swerve.hubCentricDrive;
import frc.robot.commands.swerve.intakeCentricDrive;
import frc.robot.commands.swerve.robotCentricDrive;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.shooter.Shooter_Subsystem;

public class driveController extends CommandBase {

  public enum DriveModes {
    robotCentric("Robot Centric"),
    fieldCentric("Field Centric"),
    hubCentric("Hub Centric"),
    intakeCentric("Intake Centric");
    private String name;
    private DriveModes(String name) {
      this.name = name;
    }
    public String toString() {
      return name;
    }
  }

  SwerveDrivetrain drivetrain;
  DriverControls dc;
  Shooter_Subsystem shooter;
  Magazine_Subsystem magazine;
  Limelight_Subsystem limelight;
  robotCentricDrive m_robotCentricDrive;
  fieldCentricDrive m_fieldCentricDrive;
  hubCentricDrive m_hubCentricDrive;
  intakeCentricDrive m_intakeCentricDrive;
  BasicShootCommand m_basicShootCommand;

  Command currentCmd;
  DriveModes requestedDriveMode = DriveModes.fieldCentric;
  DriveModes currentDriveMode = DriveModes.fieldCentric;
  DriveModes lastDriveMode = DriveModes.fieldCentric;
  boolean currentlyShooting = false;
  boolean shootingRequested = false;

  NetworkTable table;
  private NetworkTableEntry driveMode;
  private NetworkTableEntry shootingMode;
  public final String NT_Name = "DC"; // expose data under Drive Controller table
  int log_counter = 0;

  public driveController() {
    this.drivetrain = RobotContainer.RC().drivetrain;
    this.dc = RobotContainer.RC().driverControls;
    this.shooter = RobotContainer.RC().shooter;
    this.magazine = RobotContainer.RC().magazine;
    this.limelight = RobotContainer.RC().limelight;

    driveMode = table.getEntry("/driveMode");
    shootingMode = table.getEntry("/shootingModeOn");
  }

  @Override
  public void initialize() {
    m_robotCentricDrive = new robotCentricDrive(drivetrain, dc);
    m_fieldCentricDrive = new fieldCentricDrive(drivetrain, dc);
    m_hubCentricDrive = new hubCentricDrive(drivetrain, dc, limelight);
    m_intakeCentricDrive = new intakeCentricDrive(drivetrain, dc);
    m_basicShootCommand = new BasicShootCommand();

    currentCmd = m_fieldCentricDrive;
    CommandScheduler.getInstance().schedule(currentCmd); // start default drive mode
  }

  @Override
  public void execute() {
    checkShooter();
    checkDropout();
    checkRequests();
    updateNT();
  }

  private void checkShooter(){
    if (!currentlyShooting && shootingRequested){ //start shooting
      currentlyShooting = true;
      requestedDriveMode = DriveModes.hubCentric;
      CommandScheduler.getInstance().schedule(m_basicShootCommand);

    } else if (currentlyShooting && !shootingRequested){ //stop shooting
      currentlyShooting = false;
      requestedDriveMode = lastDriveMode;
      m_basicShootCommand.end(true);
    }
  }

  private void checkDropout(){
    if ((Math.abs(dc.getXYRotation())>0.1) && (currentDriveMode==DriveModes.intakeCentric)){
      //driver is trying to rotate while in intakeCentric, drop out of intake mode
      requestedDriveMode = DriveModes.fieldCentric;
    }
  }

  private void checkRequests(){
    if (requestedDriveMode != currentDriveMode){ //new drive mode requested
      lastDriveMode = currentDriveMode;
      currentDriveMode = requestedDriveMode;
      currentCmd.end(true); //stop prior command
      switch (currentDriveMode){
        case robotCentric:
          currentCmd = m_robotCentricDrive;
          break;
  
        case fieldCentric:
          currentCmd = m_fieldCentricDrive;
          break;    

        case hubCentric:
          currentCmd = m_hubCentricDrive;
          break;      
        
        case intakeCentric:
          currentCmd = m_intakeCentricDrive;
          break;
      }
      CommandScheduler.getInstance().schedule(currentCmd);
    }
  }

  public void cycleDriveMode() {
    //Current use case is only to allow toggling between field and intake centric
    //Make sure if in hubcentric (trigger held) that toggling drops back to default fieldcentric
    switch (currentDriveMode) {
      case robotCentric:
      case hubCentric:
      case fieldCentric:
        requestedDriveMode = DriveModes.intakeCentric;
        break;

      case intakeCentric:
        requestedDriveMode = DriveModes.fieldCentric;
        break;
    }
  }

  public void turnOnShootingMode(){
    shootingRequested = true;
  }

  public void turnOffShootingMode(){
    shootingRequested = false;
  }

  @Override
  public void end(boolean interrupted) {
    currentCmd.end(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
      // update network tables
      driveMode.setString(currentDriveMode.toString());
      shootingMode.setBoolean(currentlyShooting);
    }
  }
}
