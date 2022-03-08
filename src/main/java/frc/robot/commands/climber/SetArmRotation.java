// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.climber.ArmRotation;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;

public class SetArmRotation extends CommandBase {
  
  private double percentOutput;
  private DriverControls controls;
  private Climber climber;

  /** Creates a new SetArmRotation. */
  public SetArmRotation(DriverControls controls, Climber climber, double percentOutput) {
    this.controls = controls;
    this.climber = climber;
    this.percentOutput = percentOutput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (controls.bind(Id.Driver, XboxButton.LB).get()){
      climber.getLeftArmRotation().setPercentOutput(percentOutput);
    }
    if (controls.bind(Id.Driver, XboxButton.RB).get()){
      climber.getRightArmRotation().setPercentOutput(percentOutput);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.getLeftArmRotation().setPercentOutput(0);
    climber.getRightArmRotation().setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
