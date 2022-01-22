// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakeCommand extends CommandBase {
  /**
   * IntakeCommand is used to handle the Intake Motors, deply intake arms on Initialization
   * Constructor - public IntakeCommand(DoubleSupplier intakeSpeedFunction) 
   *    Uses the DoubleSupplier to get a "speed" value for the motor
   */

  //Defintions
  Intake_Subsystem intake;
  DoubleSupplier speedFunction;
  int intakeMode;

  //Constructor - Variable DoubleSupplier intakeSpeedFunction, a PWM "speed" for the intake Spark Motor
  public IntakeCommand(DoubleSupplier intakeSpeedFunction, int intakeMode) {
      this.intake = RobotContainer.RC().intake;
      this.intakeMode  = intakeMode;
      this.speedFunction = intakeSpeedFunction;

      addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.isDeployed() == false) {
      intake.deploy();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Call the double supplier function to get a new speed.
    if (intakeMode == 0){
      intake.on(speedFunction.getAsDouble());
    } else if(intakeMode == 1){
      intake.expell(speedFunction.getAsDouble());
    }

    //Possible TODO - check light gate and count cargo
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // don't auto-retract - drive may not want that.
    intake.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //Possible TODO - may want to finish or reset on Cargo COUNT
    return false;  // never finishes, this can be a default command
  }
}
