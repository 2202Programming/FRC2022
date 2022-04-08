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
import frc.robot.commands.MagazineController;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.MoveIntake.DeployMode;
import frc.robot.commands.Shoot.VelShootGatedCommand;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;

public class auto_cmd_group2 extends SequentialCommandGroup {
  SwerveDrivetrain m_drivetrain;
  Magazine_Subsystem m_magazine;
  Intake_Subsystem m_intake;
  DriverControls m_controls;

  public auto_cmd_group2() {
    this.m_drivetrain = RobotContainer.RC().drivetrain;
    this.m_magazine = RobotContainer.RC().magazine;
    this.m_intake = RobotContainer.RC().intake;
    this.m_controls = RobotContainer.RC().driverControls;


    Command finalAuto;
    Command feed = RobotContainer.RC().m_driveController.magazineController.getFeedCmd();
    
    if(m_controls.readSideboard(SBButton.Sw11)){
      finalAuto = auto_pathPlanner_cmd.PathFactory(1,1, "AutoPath1");
    }
    else if(m_controls.readSideboard(SBButton.Sw12)){
      finalAuto = auto_pathPlanner_cmd.PathFactory(1,1, "AutoPath2");
    }
    else{
      finalAuto = auto_pathPlanner_cmd.PathFactory(1,1, "AutoPath3");
    }
    
    addCommands(
      new MoveIntake(DeployMode.Deploy),
      new IntakeCommand(IntakeMode.InstantLoad),
      finalAuto,

      // new ParallelDeadlineGroup( //all run at same time; group ends when 1st command ends
      //   finalAuto,
      //   new IntakeCommand(IntakeMode.LoadCargo)
      // ),
      new IntakeCommand(IntakeMode.Stop),
      new MoveIntake(DeployMode.Retract),
      feed.withTimeout(0.1),
      //new VelShootCommand().withTimeout(1.8)
      new VelShootGatedCommand(new ShooterSettings(Shooter.autoVelocity-2, 0.0, 0, Shooter.DefaultRPMTolerance), RobotContainer.RC().m_driveController.magazineController).withTimeout(1.8)
    );
  }

}
