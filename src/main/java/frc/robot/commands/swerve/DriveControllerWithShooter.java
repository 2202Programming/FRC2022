// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.NTStrings;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.MagazineController;
import frc.robot.commands.Shoot.VelShootGatedCommand;

public class DriveControllerWithShooter extends DriveControllerDrivetrain {

  public MagazineController magazineController;
  VelShootGatedCommand shootCommand;

  private NetworkTableEntry NThasSolution;

  //shooting kinematics
  double Vb;
  double ToF;
  

  public DriveControllerWithShooter(MagazineController magazineController)  {
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
  public void execute() {
    checkTip();
    setVelocityOffset();
    checkShooter();
    checkDropout();
    checkRequests();
    updateNT();
  }

  //Check if we are ready to shoot (LL has target) AND driver wants to shoot
  //change drive mode to hub centric and start shooting command if driver ready to shoot and we see a LL target
  private void checkShooter(){
    if (!currentlyShooting && shootingRequested && limelight.getTarget()){ //start shooting if requested and limelight has target
      currentlyShooting = true;
      requestedDriveMode = DriveModes.hubCentric;
      CommandScheduler.getInstance().schedule(shootCommand); //right now just use fixed velocity; eventually replace with limelight distance estimated velocity
    } else if (currentlyShooting && (!shootingRequested || !limelight.getTarget())){ //stop shooting
      currentlyShooting = false;
      requestedDriveMode = lastDriveMode; //no LL target means hub centric won't work
      CommandScheduler.getInstance().cancel(shootCommand);
    } 
    //currentlyshooting implies driver believes you are generally facing LL; don't want a random light to trigger a pose update
    if (currentlyShooting && limelight.getTarget()) estimatePoseFromLimelight(); //if shooting and LL sees target, update pose
  }

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
      // update network tables
      NThasSolution.setBoolean(isOnTarget());
      driveMode.setString(currentDriveMode.toString());
    }
  }


  //If limelight has target and is locked on, update odometery pose with new estimate of position
  public void estimatePoseFromLimelight(){
    double distance = limelight.estimateDistance(); //distance from hub in meters
    double LLangle = RobotContainer.RC().limelight.getX();

    Rotation2d correctionVectorRot = RobotContainer.RC().drivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(90));
   
    //correction vector if LL angle is not zero
    //Mag is distance * sine angle
    //angle is correctionVectorRot
    Translation2d correctionVector = new Translation2d(distance * Math.sin(Math.toRadians(LLangle)),correctionVectorRot);

    //vector from robot to hub
    Translation2d hubVector = new Translation2d(distance, drivetrain.getPose().getRotation()); 
    
    hubVector.plus(correctionVector);

    //vector from origin to hub
    Translation2d hubCenter = new Translation2d(Constants.Autonomous.hubPose.getX(), Constants.Autonomous.hubPose.getY());

    //add together to get vector from origin to robot?
    hubVector.minus(hubCenter);
    Pose2d updatedPose = new Pose2d(hubVector.getX(), hubVector.getY(), drivetrain.getPose().getRotation());

    System.out.println("Original Pose: " + drivetrain.getPose());
    System.out.println("Updated Pose: " + updatedPose);
    System.out.println("Update occurred during a LL error of: " + LLangle);

    //reset pose with new X Y estimate, don't change heading
    drivetrain.setPose(updatedPose);
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
    HANGTIME = getHangTime2(distance);
    double perpendicularVelocity = -u[1]; //inverted since we shoot out the back of the robot so left/right is reversed
    double parallelVelocity = -u[0]; //inverted since we shoot out the back of the robot so forward/back is reversed

    //Adjust hangtime based on a our curve and dist estimates.
    for (int i=0; i < 5; i++) {
      perpendicularDriftDistance = perpendicularVelocity * HANGTIME; // horizontal drift distance given perpendicular velocity and hang time
      parallelDriftDistance = parallelVelocity * HANGTIME; // drift distance to/away from target given parellel velocity and hang time
      HANGTIME = getHangTime2(distance - parallelDriftDistance);
    }
    Rotation2d LLAngleOffset = new Rotation2d(Math.atan2(perpendicularDriftDistance, distance));  //angle offset of LL given known drift distance and distance to hub
    
    double parallelMagicNumber = 1.0;
    double perpendicularMagicNumber = 0.7;

    m_hubCentricDrive.setLimelightTarget(-LLAngleOffset.getDegrees() * perpendicularMagicNumber); //sign? Units should be degrees offset angle
    shootCommand.setdistanceOffset(-parallelDriftDistance * parallelMagicNumber); //add drift distance in parallel direction to calculated distance, in meters.

    //Debug prints
    // SmartDashboard.putNumber("perpendicularVelocity", perpendicularVelocity);
    // SmartDashboard.putNumber("parallelVelocity", parallelVelocity);
    // SmartDashboard.putNumber("perpendicularDriftDistance", perpendicularDriftDistance);
    // SmartDashboard.putNumber("parallelDriftDistance", -parallelDriftDistance * parallelMagicNumber);
    // SmartDashboard.putNumber("LLAngleOffset", -LLAngleOffset.getDegrees() * perpendicularMagicNumber);
    // SmartDashboard.putNumber("Hangtime", HANGTIME);
    // SmartDashboard.putNumber("LL Distance", distance);
    // SmartDashboard.putNumber("TOF", this.ToF);
    // SmartDashboard.putNumber("Vball", this.Vb);
  }

  double getHangtime(double distance){
    return (distance-1.73) / 1.4;
  }

  double getHangTime2(double d) {
    final double  g = 9.81; // [m/s^2]
    final double  th = 73.0*Math.PI/180.0;  //[rad] shooter angle
    final double  h = 2.44 - .45;  //[m] goal h - shooter h
    final double  c_th = Math.cos(th);
    final double  c_th2 = c_th*c_th;
    final double  t_th = Math.tan(th);

    //solve for V of ball as function of geometry and distance, d
    double V2 = g*d*d/((d*t_th - h)*2*c_th2);
    double v = Math.sqrt(V2);
    
    //time of flight = dist
    double tof = d/(v*c_th);
    //for dashboard
    this.ToF = tof;
    this.Vb = v;
    return tof;
  }


}
