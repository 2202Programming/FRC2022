// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.shooter.Shooter_Subsystem;

public class TestShoot extends CommandBase {

  Shooter_Subsystem shooter;
  
  /** Creates a new TestShoot. */
  public TestShoot(Shooter_Subsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Upper RPMS", 0);
    SmartDashboard.putNumber("Lower RPMS", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upper = SmartDashboard.getNumber("Upper RPMS", 0);
    double lower = SmartDashboard.getNumber("Lower RPMS", 0);
  
    shooter.setMotors(upper, lower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setMotors(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
