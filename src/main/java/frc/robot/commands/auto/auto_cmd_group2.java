// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MagazineCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.commands.MagazineCommand.MagazineMode;
import frc.robot.commands.Shoot.BasicShootCommand;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.ifx.DriverControls;

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
    
    addCommands(
      new InstantCommand( m_intake::deploy ),
      new ParallelDeadlineGroup( //all run at same time; group ends when 1st command ends
        new ParallelCommandGroup( // all conditions run at once; only one should actually drive the path
          new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isRedPath1),
          new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath2"), new WaitCommand(0), this::isRedPath2),
          new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath3"), new WaitCommand(0), this::isRedPath3),
          new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath4"), new WaitCommand(0), this::isBluePath1),
          new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath5"), new WaitCommand(0), this::isBluePath2),
          new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath6"), new WaitCommand(0), this::isBluePath3)
        ),
        new IntakeCommand((()-> 0.47), ()-> 0.20,  IntakeMode.LoadCargo),
        new MagazineCommand((()-> 1.0), MagazineMode.LoadCargo)
      ),
      new BasicShootCommand().withTimeout(10)
    );
  }

  public boolean isRedPath1(){
    System.out.println("Switch 11: " + m_controls.readSideboard(SBButton.Sw11));
    return m_controls.readSideboard(SBButton.Sw11);
  }

  public boolean isRedPath2(){
    System.out.println("Switch 12: " + m_controls.readSideboard(SBButton.Sw12));
    return m_controls.readSideboard(SBButton.Sw12);
  }

  public boolean isRedPath3(){
    System.out.println("Switch 13: " + m_controls.readSideboard(SBButton.Sw13));
    return m_controls.readSideboard(SBButton.Sw13);
  }

  public boolean isBluePath1(){
    System.out.println("Switch 21: " + m_controls.readSideboard(SBButton.Sw21));
    return m_controls.readSideboard(SBButton.Sw21);
  }

  public boolean isBluePath2(){
    System.out.println("Switch 22: " + m_controls.readSideboard(SBButton.Sw22));
    return m_controls.readSideboard(SBButton.Sw22);
  }

  public boolean isBluePath3(){
    System.out.println("Switch 23: " + m_controls.readSideboard(SBButton.Sw23));
    return m_controls.readSideboard(SBButton.Sw23);
  }
}
