// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.subsystems.hid.XboxPOV;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.climber.Climber;

public class PitAlignClimber extends CommandBase {
  Climber climber;
  DriverControls dc;
  double extRate;
  double rotRate;
  DriverControls.Id id;
  POVButton up;
  POVButton down, left, right;

  /** Creates a new PitAlignClimber. */
  public PitAlignClimber(DriverControls dc, DriverControls.Id id, Climber climber, double extRate, double rotRate) {
    this.climber = climber;
    this.dc = dc;
    this.id = id;
    up = dc.bind(this.id, XboxPOV.POV_UP);
    down = dc.bind(this.id, XboxPOV.POV_DOWN);
    left = dc.bind(this.id, XboxPOV.POV_LEFT);
    right = dc.bind(this.id, XboxPOV.POV_RIGHT);
    this.extRate = extRate;
    this.rotRate = rotRate;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setArmSync(false);
    climber.setOuterLoop(false); //using velocity
    climber.setStartingPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double v_lt = 0.0;
    double v_rt = 0.0;
    double rot_lt = 0.0;
    double rot_rt = 0.0;

    if (up.get()) {
      v_lt = dc.bind(this.id, XboxButton.LB).get() ? extRate : 0.0;
      v_rt = dc.bind(this.id, XboxButton.RB).get() ? extRate : 0.0;
    }
    if (down.get()) {
      v_lt = dc.bind(this.id, XboxButton.LB).get() ? -extRate : 0.0;
      v_rt = dc.bind(this.id, XboxButton.RB).get() ? -extRate : 0.0;
    }

    if (left.get()) {
      rot_lt = dc.bind(this.id, XboxButton.LB).get() ? rotRate : 0.0;
      rot_rt = dc.bind(this.id, XboxButton.RB).get() ? rotRate : 0.0;
    }
    if (right.get()) {
      rot_lt = dc.bind(this.id, XboxButton.LB).get() ? -rotRate : 0.0;
      rot_rt = dc.bind(this.id, XboxButton.RB).get() ? -rotRate : 0.0;
    }
    climber.setExtSpeed(v_lt, v_rt);
    climber.setRotSpeed(rot_lt, rot_rt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setStartingPos();
    climber.hold();
    System.out.println("Climber arms are set to starting position. I hope you were paying attention.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
