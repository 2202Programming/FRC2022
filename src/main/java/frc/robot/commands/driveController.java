// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerve.fieldCentricDrive;
import frc.robot.commands.swerve.hubCentricDrive;
import frc.robot.commands.swerve.intakeCentricDrive;
import frc.robot.commands.swerve.robotCentricDrive;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

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
  robotCentricDrive m_robotCentricDrive;
  fieldCentricDrive m_fieldCentricDrive;
  hubCentricDrive m_hubCentricDrive;
  intakeCentricDrive m_intakeCentricDrive;
  Command currentCmd;
  DriveModes requestedDriveMode = DriveModes.fieldCentric;
  DriveModes currentDriveMode = DriveModes.fieldCentric;
  DriveModes lastDriveMode = DriveModes.fieldCentric;
  boolean currentlyShooting = false;

  public driveController(SwerveDrivetrain drivetrain, DriverControls dc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.dc = dc;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotCentricDrive = new robotCentricDrive(drivetrain, dc);
    m_fieldCentricDrive = new fieldCentricDrive(drivetrain, dc);
    m_hubCentricDrive = new hubCentricDrive(drivetrain, dc);
    m_intakeCentricDrive = new intakeCentricDrive(drivetrain, dc);

    currentCmd = m_fieldCentricDrive;
    CommandScheduler.getInstance().schedule(currentCmd); // start default drive mode
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    checkDropout();
    checkRequests();
  }

  public void cycleDriveMode() {
    //Current use case is only to allow toggling between field and intake centric
    //Make sure if in hubcentric (trigger held) that toggling doesn't do anything
    switch (currentDriveMode) {

      case fieldCentric:
        requestedDriveMode = DriveModes.fieldCentric;
        break;

      case intakeCentric:
        requestedDriveMode = DriveModes.intakeCentric;
        break;
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
