// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.MagazineSettings;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakeCommand extends CommandBase {
  /**
   * IntakeCommand is used to handle the Intake Motors (either intaking or expelling), deploy intake arms on Initialization
   */

  //Defintions
  Intake_Subsystem intake;
  DoubleSupplier intakeSpeed;
  DoubleSupplier sideIntakeSpeed;
  boolean finished = false;

  public enum IntakeMode {
    LoadCargo, ExpellCargo, Stop, InstantLoad
  }
  IntakeMode mode;

  /**
   * Constructor
   * @param intakeSpeedFunction - a PWM "speed" for the intake Spark Motor
   * @param intakeMode - determines if we are intaking or expelling cargo
   */
  public IntakeCommand(DoubleSupplier intakeSpeedFunction, DoubleSupplier sideIntakeSpeedFunction, IntakeMode mode) {
      this.intake = RobotContainer.RC().intake;
      this.mode  = mode;
      this.intakeSpeed = intakeSpeedFunction;
      this.sideIntakeSpeed = sideIntakeSpeedFunction;

      //addRequirements(intake);
  }

  public IntakeCommand(IntakeMode mode) {
    this(()->MagazineSettings.defaultFrontIntakeSpeed, ()->MagazineSettings.defaultSideIntakeSpeed, mode);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (intake.isDeployed() == false) {
    //   intake.deploy();
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Call the double supplier function to get a new speed.
    switch(mode){
      case LoadCargo:
        intake.on(intakeSpeed.getAsDouble(),sideIntakeSpeed.getAsDouble());
        break;
      case ExpellCargo:
        intake.on( -intakeSpeed.getAsDouble(),sideIntakeSpeed.getAsDouble());
        break;
      case InstantLoad: //for command groups that need an instant command to finish but leave intake running
        intake.on(intakeSpeed.getAsDouble(),sideIntakeSpeed.getAsDouble());
        finished = true;
        break;
      case Stop:
        intake.off();
        finished = true;
        break;
    }

    //Possible TODO - check light gate and count cargo
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // don't auto-retract - drive may not want that.
    intake.off();
  }

  public void setFinished(){
    finished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //Possible TODO - may want to finish or reset on Cargo COUNT
    return finished;  // never finishes, this can be a default command
  }
}
