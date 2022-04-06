// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Shooter;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveIntake.DeployMode;
import frc.robot.commands.Shoot.LimeLightAim;
import frc.robot.commands.Shoot.VelShootCommand;
import frc.robot.commands.Shoot.VelShootGatedCommand;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;

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
      finalAuto = auto_pathPlanner_cmd.PathFactory2(3,2, "Auto21");
      finalAutoB = auto_pathPlanner_cmd.PathFactory3(3,2, "Auto21B");
    }
    else if(m_controls.readSideboard(SBButton.Sw12)){
      finalAuto = auto_pathPlanner_cmd.PathFactory2(3,2, "Auto22");
      finalAutoB = auto_pathPlanner_cmd.PathFactory3(3,2, "Auto22B");
    }
    else{
      finalAuto = auto_pathPlanner_cmd.PathFactory2(3,2, "Auto23");
      finalAutoB = auto_pathPlanner_cmd.PathFactory3(3,2, "Auto23B");
    }
    

    addCommands(
      new MoveIntake(DeployMode.Deploy),
      new IntakeCommand(IntakeMode.InstantLoad),
      finalAuto,
      new MoveIntake(DeployMode.Deploy),
      new WaitCommand(2),
      new MoveIntake(DeployMode.Retract),
      finalAutoB,

      //if limelight is functioning well at competition, this will use LL to aim last shot since it has most odometerty drift
      //if SW16 is on it will skip
      new ConditionalCommand(new WaitCommand(0), new LimeLightAim().withTimeout(2), () -> RobotContainer.RC().driverControls.readSideboard(SBButton.Sw16)),
      
      //if limelight is functioning well at competition, this will use LL to pick velocity for last shot since it has most odometerty drift
      //if SW16 is on it will skip and just pick hard coded velocity
      new ConditionalCommand(
        new VelShootGatedCommand(new ShooterSettings(Shooter.autoVelocity-2, 0.0, 0, Shooter.DefaultRPMTolerance), RobotContainer.RC().m_driveController.magazineController),
        new VelShootGatedCommand(RobotContainer.RC().m_driveController.magazineController, RobotContainer.RC().m_driveController),
        () -> RobotContainer.RC().driverControls.readSideboard(SBButton.Sw16)
      ),
      
      new IntakeCommand(IntakeMode.Stop)
    );
  }

}