// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MagazineCommand;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.MagazineCommand.MagazineMode;
import frc.robot.commands.MoveIntake.DeployMode;
import frc.robot.commands.Shoot.BasicShootCommand;
import frc.robot.commands.Shoot.LimelightAim;
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

  public auto_cmd_group2(SwerveDrivetrain m_drivetrain, Magazine_Subsystem m_magazine, Intake_Subsystem m_intake, DriverControls m_controls) {
    this.m_drivetrain = m_drivetrain;
    this.m_magazine = m_magazine;
    this.m_intake = m_intake;
    this.m_controls = m_controls;

    Command finalAuto;

    if(m_controls.readSideboard(SBButton.Sw11)){
      finalAuto = auto_pathPlanner_cmd.PathFactory(m_drivetrain, "AutoPath6");
    }
    else if(m_controls.readSideboard(SBButton.Sw12)){
      finalAuto = auto_pathPlanner_cmd.PathFactory(m_drivetrain, "AutoPath5");
    }
    else{
      finalAuto = auto_pathPlanner_cmd.PathFactory(m_drivetrain, "AutoPath4");
    }
    

    addCommands(
      new MoveIntake(DeployMode.Deploy),
      new InstantCommand( RobotContainer.RC().limelight::enableLED ),
      new ParallelDeadlineGroup( //all run at same time; group ends when 1st command ends
        finalAuto,
        new IntakeCommand((()-> 0.55), ()-> 0.20,  IntakeMode.LoadCargo),
        new MagazineCommand((()-> 1.0), MagazineMode.LoadCargo)
      ),
      new MoveIntake(DeployMode.Retract),
      new ParallelDeadlineGroup( //all run at same time; group ends when 1st command ends
        new LimelightAim(1.0).withTimeout(3),
        new IntakeCommand((()-> 0.55), ()-> 0.20,  IntakeMode.LoadCargo),
        new MagazineCommand((()-> 1.0), MagazineMode.LoadCargo)
    ),
      new MagazineCommand((()-> 1.0), MagazineMode.ExpellCargo).withTimeout(.75),
      new ParallelDeadlineGroup(
        new BasicShootCommand(new ShooterSettings(20.0, 0.0), 20).withTimeout(10),
        new MagazineCommand((()-> 1.0), MagazineMode.LoadCargo),
        new IntakeCommand((()-> 0.55), ()-> 0.20,  IntakeMode.LoadCargo)
      )
    );
  }

}
