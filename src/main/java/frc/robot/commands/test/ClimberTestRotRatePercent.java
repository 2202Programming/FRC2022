// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

/**
 * Uses velocity mod to tune hardware pid loops
 */

public class ClimberTestRotRatePercent extends CommandBase {
  Climber climber;
  double pct_l;
  double pct_r;
  double min_ext;
  double max_ext;
  int delay_l;
  int delay_r;
  boolean syncMode = false;
  final double delayCount = 20;

  public ClimberTestRotRatePercent(Climber climber, double pct, double min_ext, double max_ext) {
    this.climber = climber;
    this.min_ext = min_ext;
    this.max_ext = max_ext;
    pct_l = pct;
    pct_r = pct;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delay_l = 0;
    delay_r = 0;
    climber.setRotSpeed(pct_l, pct_r);
    climber.setArmSync(syncMode);
    climber.setOuterLoop(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // change the direction at the end
    if (((climber.getLeftArmRotation().getRotationDegrees() >= max_ext) || 
         climber.getLeftArmRotation().getRotationDegrees() <= min_ext) && delay_l > delayCount) {
      pct_l *= -1.0;
      delay_l = 0;

    }
    if (((climber.getRightArmRotation().getRotationDegrees() >= max_ext) || 
      climber.getRightArmRotation().getRotationDegrees() <= min_ext) && delay_r > delayCount) {
      pct_r *= -1.0;
      delay_r = 0;
    }

    if (syncMode == true) {
      pct_r = pct_l;
    }
    climber.setPercentOutputRot(pct_l, pct_r);
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
