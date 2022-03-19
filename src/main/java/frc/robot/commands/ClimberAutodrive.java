// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sensors_Subsystem;

public class ClimberAutodrive extends CommandBase {
  /** Creates a new ClimberAutodrive. */
  Sensors_Subsystem ColorSensor;
  boolean finished;
  public ClimberAutodrive(Sensors_Subsystem ColorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ColorSensor = ColorSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ColorSensor.getColor() != "black"){
      //TODO drive forward
    } else {
      //TODO Stop driving
      this.end(true);
    }
  }
  public void setFinished(){
    finished = true;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
