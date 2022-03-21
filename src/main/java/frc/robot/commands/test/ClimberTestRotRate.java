// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

/**
 * Uses velocity mod to tune hardware pid loops
 */

public class ClimberTestRotRate extends CommandBase {
  Climber climber;
  double speed_l;
  double speed_r;
  double min_ext;
  double max_ext;
  int delay_l;
  int delay_r;
  boolean syncMode = true;
  final double delayCount = 20;

  public ClimberTestRotRate(Climber climber, double spd, double min_ext, double max_ext) {
    this.climber = climber;
    this.min_ext = min_ext;
    this.max_ext = max_ext;
    speed_l = spd;
    speed_r = spd;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setOuterLoop(false);
    delay_l = 0;
    delay_r = 0;
    climber.setRotSpeed(speed_l, speed_r);
    climber.setArmSync(syncMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // change the direction at the end
    if (((climber.getLeftArmRotation().getRotationDegrees() >= max_ext) || 
         climber.getLeftArmRotation().getRotationDegrees() <= min_ext) && delay_l > delayCount) {
      speed_l *= -1.0;
      delay_l = 0;

    }
    if (((climber.getRightArmRotation().getRotationDegrees() >= max_ext) || 
      climber.getRightArmRotation().getRotationDegrees() <= min_ext) && delay_r > delayCount) {
      speed_r *= -1.0;
      delay_r = 0;
    }

    if (syncMode == true) {
      speed_r = speed_l;
    }
    climber.setRotSpeed(speed_l, speed_r);
    delay_l++;
    delay_r++;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setRotSpeed(0.0);
    climber.setOuterLoop(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
