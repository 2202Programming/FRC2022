// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MagazineCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.MagazineCommand.MagazineMode;
import frc.robot.subsystems.shooter.Shooter_Subsystem;

public class SuperDuperDumbShooter extends CommandBase {
  //failsafe shooter command - run intake and magazine at default intake speeds, and run shooter wheels at 100%

  double percent;
  Shooter_Subsystem shooter;
  IntakeCommand intakeCommand;
  MagazineCommand magazineCommand;

  public SuperDuperDumbShooter(double percent) {
    this.shooter = RobotContainer.RC().shooter;
    this.percent = percent;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    intakeCommand = new IntakeCommand(IntakeMode.LoadCargo);
    magazineCommand = new MagazineCommand(MagazineMode.LoadCargo);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //CommandScheduler.getInstance().schedule(intakeCommand);
    //CommandScheduler.getInstance().schedule(magazineCommand);
    shooter.onPercent(percent, percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.off();
    intakeCommand.setFinished();
    magazineCommand.setFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
