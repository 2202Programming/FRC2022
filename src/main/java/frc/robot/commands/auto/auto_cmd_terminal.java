// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.MoveIntake.DeployMode;
import frc.robot.commands.Shoot.VelShootCommand;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.ifx.DriverControls;

public class auto_cmd_terminal extends SequentialCommandGroup {
  SwerveDrivetrain m_drivetrain;
  Magazine_Subsystem m_magazine;
  Intake_Subsystem m_intake;
  DriverControls m_controls;

  public auto_cmd_terminal() {
    this.m_drivetrain = RobotContainer.RC().drivetrain;
    this.m_magazine = RobotContainer.RC().magazine;
    this.m_intake = RobotContainer.RC().intake;
    this.m_controls = RobotContainer.RC().driverControls;


    Command finalAuto;
    Command finalAutoB = new WaitCommand(0);

    if(m_controls.readSideboard(SBButton.Sw11)){
      finalAuto = auto_pathPlanner_cmd.PathFactory2(2.5,2, "Auto21");
      finalAutoB = auto_pathPlanner_cmd.PathFactory3(2.5,2, "Auto21B");
    }
    else if(m_controls.readSideboard(SBButton.Sw12)){
      finalAuto = auto_pathPlanner_cmd.PathFactory2(2.5,2, "Auto22");
      finalAutoB = auto_pathPlanner_cmd.PathFactory3(2.5,2, "Auto22B");
    }
    else{
      finalAuto = auto_pathPlanner_cmd.PathFactory2(2.5,2, "Auto23");
      finalAutoB = auto_pathPlanner_cmd.PathFactory3(2.5,2, "Auto23B");
    }
    

    addCommands(
      new MoveIntake(DeployMode.Deploy),
      new IntakeCommand(IntakeMode.InstantLoad),
      finalAuto,
      new MoveIntake(DeployMode.Deploy),
      new WaitCommand(2),
      finalAutoB,
      new MoveIntake(DeployMode.Retract),
      new ConditionalCommand(new VelShootCommand(Shooter.autoVelocity - 2, false).withTimeout(2.5), new VelShootCommand().withTimeout(2.5), () -> RobotContainer.RC().driverControls.readSideboard(SBButton.Sw16)), //decide wether or not to use limelight for shooting final shot in auto
      new IntakeCommand(IntakeMode.Stop)
    );
  }

}