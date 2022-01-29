// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;

public class TestShoot extends CommandBase {
  double lastJoint = 0;
  Shooter_Subsystem shooter;

  NetworkTable table;
  NetworkTableEntry ntUpperRPM;
  NetworkTableEntry ntLowerRPM;
  NetworkTableEntry  ntBallVel;
  NetworkTableEntry ntBallRotationRPM;
  
  /** Creates a new TestShoot. */
  public TestShoot(Shooter_Subsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table = NetworkTableInstance.getDefault().getTable("TestShooter");
    ntUpperRPM = table.getEntry("UpperRPM");
    ntLowerRPM = table.getEntry("LowerRPM");
    ntBallVel = table.getEntry("BallVel");
    ntBallRotationRPM = table.getEntry("BallRotationRPM");


    ntUpperRPM.setDouble(0);
    ntLowerRPM.setDouble(0);
    ntBallVel.setDouble(0);
    ntBallRotationRPM.setDouble(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double upper = SmartDashboard.getNumber("Upper RPMS", 0);
    // double lower = SmartDashboard.getNumber("Lower RPMS", 0);

    double newJoint =  ntBallVel.getDouble(lastJoint);
    if(newJoint != lastJoint){
      // upper = newJoint;
      // lower = newJoint;
      // SmartDashboard.putNumber("Upper RPMS", newJoint);
      // SmartDashboard.putNumber("Lower RPMS", newJoint);
      lastJoint = newJoint;
      shooter.setShooterSettings(new ShooterSettings(lastJoint, ntBallRotationRPM.getDouble(0), 0, .1));
      shooter.spinupShooter();
    }
  
    // shooter.setMotors(upper, lower);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setMotors(0,0);
  }
}
