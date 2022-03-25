// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    if(m_controls.readSideboard(SBButton.Sw11)){
      finalAuto = auto_pathPlanner_cmd.PathFactory2(2.5,2, "Auto21");
    }
    else if(m_controls.readSideboard(SBButton.Sw12)){
      finalAuto = auto_pathPlanner_cmd.PathFactory2(2.5,2, "Auto22");
    }
    else{
      finalAuto = auto_pathPlanner_cmd.PathFactory2(2.5,2, "Auto23");
    }
    

    addCommands(
      new MoveIntake(DeployMode.Deploy),
      new ParallelDeadlineGroup( //all run at same time; group ends when 1st command ends
        finalAuto,
        new IntakeCommand(IntakeMode.LoadCargo)
      ),
      new IntakeCommand(IntakeMode.Stop),
      new MoveIntake(DeployMode.Retract),
      new VelShootCommand(Shooter.autoVelocity, false).withTimeout(2.5)
    );
  }

}
