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
  int delay_l;
  int delay_r;
  boolean syncMode = true;
  final double delayCount = 10;
  boolean move_to_done = false;


  public ClimberTestVelocity(Climber climber, double spd, double min_ext, double max_ext) {
    this.climber = climber;
    this.min_ext = min_ext;
    this.max_ext = max_ext;
    speed_l = Math.abs(spd);
    speed_r = speed_l;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delay_l = 0;
    delay_r = 0;
    climber.setOuterLoop(true);
    climber.setExtension(min_ext + 0.5);
    climber.setArmSync(syncMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!move_to_done) {
      //wait until extension is in bound
      if (climber.outerLoopExtDone()) {
        move_to_done = true;
        climber.setOuterLoop(false);  //use pure velocity control
      }
      else return;
    }

    // change the direction at the end
    if (((climber.getLeftArmExtension().getInches() >= max_ext) || 
         climber.getLeftArmExtension().getInches() <= min_ext) && delay_l > delayCount) {
      speed_l *= -1.0;
      delay_l = 0;

    }
    if (((climber.getRightArmExtension().getInches() >= max_ext) || 
      climber.getRightArmExtension().getInches() <= min_ext) && delay_r > delayCount) {
      speed_r *= -1.0;
      delay_r = 0;
    }

    if (syncMode == true) {
      speed_r = speed_l;
    }
    climber.setExtSpeed(speed_l, speed_r);
    delay_l++;
    delay_r++;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setExtSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
