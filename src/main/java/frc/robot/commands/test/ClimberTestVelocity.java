// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

/**
 * Uses velocity mod to tune hardware pid loops
 */

public class ClimberTestVelocity extends CommandBase {
  Climber climber;
  double speed_l;
  double speed_r;
  double min_ext;
  double max_ext;

  public ClimberTestVelocity(Climber climber, double spd, double min_ext, double max_ext) {
    this.climber = climber;
    this.min_ext = min_ext;
    this.max_ext = max_ext;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setStartingPos();   //assume zero
    climber.setExtSpeed(speed_l, speed_r);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // change the direction at the end
    if ((climber.getLeftArmExtension().getInches() >= max_ext) || 
         climber.getLeftArmExtension().getInches() <= min_ext) {
      speed_l *= -1.0;
    }
    if ((climber.getRightArmExtension().getInches() >= max_ext) || 
      climber.getRightArmExtension().getInches() <= min_ext) {
      speed_r *= -1.0;
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
