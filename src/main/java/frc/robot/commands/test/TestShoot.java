// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;

public class TestShoot extends CommandBase {
  final double TESTANGLE = 0.0;
  final double TESTTOL = 0.02;

  Shooter_Subsystem shooter;

  NetworkTable table;
  NetworkTableEntry ntUpperRPM;   //FW speeds (output)
  NetworkTableEntry ntLowerRPM;
  NetworkTableEntry ntBallVel;    // ball physics (input) 
  NetworkTableEntry ntBallRPS;
  
  ShooterSettings  cmdSS;         // settings to drive shooter with
  
  /** Creates a new TestShoot. */
  public TestShoot(Shooter_Subsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
  
    table = NetworkTableInstance.getDefault().getTable("TestShooter");
    ntUpperRPM = table.getEntry("UpperRPM");
    ntLowerRPM = table.getEntry("LowerRPM");
    ntBallVel = table.getEntry("BallVel");
    ntBallRPS = table.getEntry("BallRPS");

    ntUpperRPM.setDouble(0);
    ntLowerRPM.setDouble(0);
    ntBallVel.setDouble(0);
    ntBallRPS.setDouble(0);
    cmdSS = new ShooterSettings(ntBallVel.getDouble(0.0), ntBallRPS.getDouble(0.0), TESTANGLE, TESTTOL);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //read network for new cmd values
    cmdSS.vel = ntBallVel.getDouble(cmdSS.vel);
    cmdSS.rps = ntBallRPS.getDouble(cmdSS.rps);
    ShooterSettings currentSS = shooter.getShooterSettings();

    //any difference? tell the shoother to spinup to new cmd settings
    if (!cmdSS.equals(currentSS)) {
      shooter.spinup(cmdSS);
    }

    //shooter.onPercent(20, 20);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.off();
  }
}
