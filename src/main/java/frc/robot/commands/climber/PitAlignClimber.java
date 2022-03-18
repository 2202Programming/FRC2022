// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class PitAlignClimber extends CommandBase {
  Climber climber;
  DriverControls dc;
  double extRate;
  double rotRate;

  /** Creates a new PitAlignClimber. */
  public PitAlignClimber(DriverControls dc, Climber climber, double extRate, double rotRate) {
    this.climber = climber;
    this.dc = dc;
    addRequirements(climber);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double v_lt = dc.bind(Id.Driver, XboxButton.LB).get() ? extRate : 0.0;
    double v_rt = dc.bind(Id.Driver, XboxButton.RB).get() ? extRate : 0.0;


    climber.setExtSpeed(v_lt, v_rt);
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
