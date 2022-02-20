// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.BasicShootCommand;
import frc.robot.commands.ResetPosition;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.ifx.DriverControls;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class auto_cmd_group extends SequentialCommandGroup {
  /** Creates a new auto_cmd_group. */
  SwerveDrivetrain m_drivetrain;
  Magazine_Subsystem m_magazine;
  Intake_Subsystem m_intake;
  DriverControls m_controls;

  public auto_cmd_group(SwerveDrivetrain m_drivetrain, Magazine_Subsystem m_magazine, Intake_Subsystem m_intake, DriverControls m_controls) {
    this.m_drivetrain = m_drivetrain;
    this.m_magazine = m_magazine;
    this.m_intake = m_intake;
    this.m_controls = m_controls;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(m_intake::defaultOn),
      new InstantCommand(m_intake::deploy),
      new ConditionalCommand(new ResetPosition(new Pose2d(Constants.Autonomous.RED_START_A_X, Constants.Autonomous.RED_START_A_Y, Constants.Autonomous.RED_START_A_ROT), m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isRedPath1),
      new ConditionalCommand(new ResetPosition(new Pose2d(Constants.Autonomous.RED_START_B_X, Constants.Autonomous.RED_START_B_Y, Constants.Autonomous.RED_START_A_ROT), m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isRedPath2),
      new ConditionalCommand(new ResetPosition(new Pose2d(Constants.Autonomous.RED_START_C_X, Constants.Autonomous.RED_START_C_Y, Constants.Autonomous.RED_START_A_ROT), m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isRedPath3),
      new ConditionalCommand(new ResetPosition(new Pose2d(Constants.Autonomous.BLUE_START_A_X, Constants.Autonomous.BLUE_START_A_Y, Constants.Autonomous.RED_START_A_ROT), m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isBluePath1),
      new ConditionalCommand(new ResetPosition(new Pose2d(Constants.Autonomous.BLUE_START_B_X, Constants.Autonomous.BLUE_START_B_Y, Constants.Autonomous.RED_START_A_ROT), m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isBluePath2),
      new ConditionalCommand(new ResetPosition(new Pose2d(Constants.Autonomous.BLUE_START_C_X, Constants.Autonomous.BLUE_START_C_Y, Constants.Autonomous.RED_START_A_ROT), m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isBluePath3),
      new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isRedPath1),
      new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath2"), new WaitCommand(0), this::isRedPath2),
      new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath3"), new WaitCommand(0), this::isRedPath3),
      new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath4"), new WaitCommand(0), this::isBluePath1),
      new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath5"), new WaitCommand(0), this::isBluePath2),
      new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath6"), new WaitCommand(0), this::isBluePath3)

      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath1"), null, this::isRedPath1),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath2"), null, this::isRedPath2),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath3"), null, this::isRedPath3),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath4"), null, this::isBluePath1),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath5"), null, this::isBluePath2),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath6"), null, this::isBluePath3),

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
