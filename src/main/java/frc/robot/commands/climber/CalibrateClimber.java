// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.climber.ArmExtension;
import frc.robot.subsystems.climber.ArmRotation;

public class CalibrateClimber extends CommandBase {
  
  private ArmExtension leftExt;
  private ArmExtension rightExt;
  private ArmRotation leftRot;
  private ArmRotation rightRot;

  /** Creates a new CalibrateClimber. */
  public CalibrateClimber(Climber climber) {
    leftExt = climber.getLeftArmExtension();
    rightExt = climber.getRightArmExtension();
    leftRot = climber.getLeftArmRotation();
    rightRot = climber.getRightArmRotation();
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //move EVERYTHING slowly until the limit switches are hit
    leftExt.setPercentOutput(-0.2);
    rightExt.setPercentOutput(-0.2);

    // Rotation only has a forward limit switch
    leftRot.setPercentOutput(0.25);
    rightRot.setPercentOutput(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // in a perfect world we would monitor stuff here...

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftExt.setPercentOutput(0);
    leftExt.setMotorPos(0);
    rightExt.setPercentOutput(0);
    rightExt.setMotorPos(0);

    leftRot.setPercentOutput(0);
    leftRot.setEncoderPos(0);
    rightRot.setPercentOutput(0);
    rightRot.setEncoderPos(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // done when all limit switches are hit
    // Note: this relies on the motor controller stopping the motors because 
    // this command drives each motor at a constant power.
    return (leftExt.isLowerLimitHit() && rightExt.isLowerLimitHit() && 
            leftRot.isForwardLimitHit() && rightRot.isForwardLimitHit());
  }
}
