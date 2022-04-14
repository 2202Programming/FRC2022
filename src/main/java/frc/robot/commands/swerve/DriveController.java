// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.NTStrings;
import frc.robot.RobotContainer;
import frc.robot.commands.MagazineController;
import frc.robot.commands.Shoot.SolutionProvider;
import frc.robot.commands.Shoot.VelShootGatedCommand;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.shooter.Shooter_Subsystem;

public class DriveController  extends CommandBase implements SolutionProvider {

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
  public MagazineController magazineController;
  Limelight_Subsystem limelight;
  RobotCentricDrive m_robotCentricDrive;
  FieldCentricDrive m_fieldCentricDrive;
  HubCentricDrive m_hubCentricDrive;
  IntakeCentricDrive m_intakeCentricDrive;
  VelShootGatedCommand shootCommand;

  Command currentCmd;
  DriveModes requestedDriveMode = DriveModes.fieldCentric;
  DriveModes currentDriveMode = DriveModes.fieldCentric;
  DriveModes lastDriveMode = DriveModes.fieldCentric;
  boolean currentlyShooting = false;
  boolean shootingRequested = false;
  boolean hasSolution = false;

  NetworkTable table;
  NetworkTable shooterTable;
  NetworkTable positionTable;
  private NetworkTableEntry driveMode;
  private NetworkTableEntry NThasSolution;
  public final String NT_Name = "DC"; 
  public final String NT_ShooterName = "Shooter"; 
  
  int log_counter = 0;

  public DriveController(MagazineController magazineController)  {
    System.out.println("Drive Controller Constructed");
    this.drivetrain = RobotContainer.RC().drivetrain;
    this.dc = RobotContainer.RC().driverControls;
    this.shooter = RobotContainer.RC().shooter;
    this.magazineController = magazineController;
    this.limelight = RobotContainer.RC().limelight;

    m_robotCentricDrive = new RobotCentricDrive(drivetrain, dc);
    m_fieldCentricDrive = new FieldCentricDrive(drivetrain, dc);
    m_hubCentricDrive = new HubCentricDrive(drivetrain, dc, limelight);
    m_intakeCentricDrive = new IntakeCentricDrive(drivetrain, dc);
    //shootCommand = new VelShootCommand(45,false);  //right now just use fixed velocity; eventually replace with limelight distance estimated velocity
    
    //use this one when ready for solution provider and velocity auto adjustment
    shootCommand = new VelShootGatedCommand(magazineController, this);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    shooterTable = NetworkTableInstance.getDefault().getTable(NT_ShooterName);
    positionTable = NetworkTableInstance.getDefault().getTable(NTStrings.NT_Name_Position);
    driveMode = table.getEntry("/DriveController/driveMode");
    NThasSolution = shooterTable.getEntry("/DriveController/HasSolution");
  }

  @Override
  public void initialize() {
    System.out.println("Drive Controller Initialized");
    currentCmd = m_fieldCentricDrive;
    CommandScheduler.getInstance().schedule(currentCmd); // start default drive mode
  }

  @Override
  public void execute() {
    checkTip();
    setVelocityOffset();
    checkShooter();
    checkDropout();
    checkRequests();
    updateNT();
  }

  private void checkShooter(){
    if (!currentlyShooting && shootingRequested && limelight.getTarget() && limelight.getLEDStatus()){ //start shooting if requested and limelight has target
      currentlyShooting = true;
      limelight.enableLED();
      requestedDriveMode = DriveModes.hubCentric;
      CommandScheduler.getInstance().schedule(shootCommand); //right now just use fixed velocity; eventually replace with limelight distance estimated velocity
    } else if (currentlyShooting && !shootingRequested){ //stop shooting
      currentlyShooting = false;
      limelight.disableLED();
      requestedDriveMode = lastDriveMode;
      CommandScheduler.getInstance().cancel(shootCommand);
    } 
    if (currentlyShooting) { 
        NThasSolution.setBoolean(isOnTarget());
    }
  }


  /**
   * isOnTarget provides feedback to shoot command being run.
   */
  @Override
  public boolean isOnTarget(){
    // free shoot mode - just return true 
    if (currentDriveMode == DriveModes.hubCentric)
      return m_hubCentricDrive.isReady();
    return true;   //all other modes driver is the targeting solution
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

  public void setRobotCentric() {
    requestedDriveMode = DriveModes.robotCentric;
  }

  public void setFieldCentric() {
    requestedDriveMode = DriveModes.fieldCentric;
  }

  public void turnOnShootingMode(){
    shootingRequested = true;
    limelight.enableLED();
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
    }
  }

  //for future use in autobalance
  private void checkTip(){
    double kOffBalanceAngleThresholdDegrees = 5;
    double pitchAngleDegrees = RobotContainer.RC().sensors.getPitch();    
    double rollAngleDegrees = RobotContainer.RC().sensors.getRoll();
    if (Math.abs(pitchAngleDegrees)>kOffBalanceAngleThresholdDegrees){
      //System.out.println("***PITCH WARNING: Pitch angle:"+pitchAngleDegrees);
    }
    if (Math.abs(rollAngleDegrees)>kOffBalanceAngleThresholdDegrees){
      //System.out.println("***ROLL WARNING: Roll Angle:"+rollAngleDegrees);
    }
  }

  //should estimate how many degrees to offset LL target in X direction to compensate for perpendicular velocity*hangtime
  //also calculates how much to adjust shooting distance based on robot velocity towards/away from target
  //should be run only when we are shooting and on target, so can assume we are roughly facing hub
  public void setVelocityOffset(){
    double perpendicularDriftDistance=0.0;
    double parallelDriftDistance=0.0;
    double HANGTIME;
    double distance;

    //Note: Robot coordinates are same as LL coordinates rotated 180 deg
    double[] u = {drivetrain.getChassisSpeeds().vxMetersPerSecond, drivetrain.getChassisSpeeds().vyMetersPerSecond}; 
    //in m/s - robot's direction vector (in robot frame of reference, intake is forward 0 deg)
  
    distance = limelight.estimateDistance(); //distance to target based on LL angle
    HANGTIME = getHangtime(distance);
    double perpendicularVelocity = -u[1]; //inverted since we shoot out the back of the robot so left/right is reversed
    double parallelVelocity = -u[0]; //inverted since we shoot out the back of the robot so forward/back is reversed

    //Adjust hangtime based on a our curve and dist estimates.
    for (int i=0; i < 5; i++) {
      perpendicularDriftDistance = perpendicularVelocity * HANGTIME; // horizontal drift distance given perpendicular velocity and hang time
      parallelDriftDistance = parallelVelocity * HANGTIME; // drift distance to/away from target given parellel velocity and hang time
      HANGTIME = getHangtime(distance - parallelDriftDistance);
    }
    Rotation2d LLAngleOffset = new Rotation2d(Math.atan2(perpendicularDriftDistance, distance));  //angle offset of LL given known drift distance and distance to hub
    
    double parallelMagicNumber = 0.4;
    double perpendicularMagicNumber = 0.7;

    m_hubCentricDrive.setLimelightTarget(-LLAngleOffset.getDegrees() * perpendicularMagicNumber); //sign? Units should be degrees offset angle
    shootCommand.setdistanceOffset(-parallelDriftDistance * parallelMagicNumber); //add drift distance in parallel direction to calculated distance, in meters.

    //Debug prints
    SmartDashboard.putNumber("perpendicularVelocity", perpendicularVelocity);
    SmartDashboard.putNumber("parallelVelocity", parallelVelocity);
    SmartDashboard.putNumber("perpendicularDriftDistance", perpendicularDriftDistance);
    SmartDashboard.putNumber("parallelDriftDistance", -parallelDriftDistance * parallelMagicNumber);
    SmartDashboard.putNumber("LLAngleOffset", -LLAngleOffset.getDegrees() * perpendicularMagicNumber);
    SmartDashboard.putNumber("Hangtime", HANGTIME);
    SmartDashboard.putNumber("LL Distance", distance);
  }

  private double getHangtime(double distance){
    return (distance-1.73) / 1.4;
  }
}
