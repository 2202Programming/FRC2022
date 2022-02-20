// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Positioner_Subsystem;

public class PositionerCommand extends CommandBase {
  /** Creates a new PositionerCommand. */
  final Positioner_Subsystem positioner;
  public enum PositionerMode {
    MoveUp, MoveDown
  }
  PositionerMode mode;

  public PositionerCommand(PositionerMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.positioner = RobotContainer.RC().positioner;
    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      // positioner.downward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //   switch (mode){
    //     case MoveUp:
    //       if (!positioner.isUpPosition())
    //       {
    //         positioner.upward();
    //       }
    //       break;

    //     case MoveDown:
    //       if (positioner.isUpPosition())
    //       {
    //         positioner.downward();
    //       }
    //       break;
    //   }
    
    
    // if (mode == PositionerMode.MoveUp){
    //     positioner.upward();
    // }else if (mode == PositionerMode.MoveDown){
    //     positioner.downward();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (positioner.isUpPosition()) {
    //   positioner.downward();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
