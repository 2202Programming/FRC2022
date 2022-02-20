// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.BasicShootCommand;
import frc.robot.commands.ResetPosition;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.hid.SideboardController.SBButton;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class auto_cmd_group extends SequentialCommandGroup {
  /** Creates a new auto_cmd_group. */
  SwerveDrivetrain m_drivetrain;
  Magazine_Subsystem m_magazine;
  Intake_Subsystem m_intake;

  public auto_cmd_group(SwerveDrivetrain m_drivetrain, Magazine_Subsystem m_magazine, Intake_Subsystem m_intake) {
    this.m_drivetrain = m_drivetrain;
    this.m_magazine = m_magazine;
    this.m_intake = m_intake;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetPosition(new Pose2d(Constants.Autonomous.RED_START_A_X, Constants.Autonomous.RED_START_A_Y, Constants.Autonomous.RED_START_A_ROT), m_drivetrain, "None"), //CHANGE
      new InstantCommand(m_intake::defaultOn),
      new InstantCommand(m_intake::deploy),
      new auto_pathPlanner_cmd(m_drivetrain, "AutoPath1") //CHANGE
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath1"), null, this::isRedPath1),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath2"), null, this::isRedPath2),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath3"), null, this::isRedPath3),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath4"), null, this::isBluePath1),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath5"), null, this::isBluePath2),
      // new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath6"), null, this::isBluePath3),

    );
  }

  public boolean isRedPath1(){
    return RobotContainer.auto_path_name.equals("AutoPath1");
  }

  public boolean isRedPath2(){
    return RobotContainer.auto_path_name.equals("AutoPath2");
  }

  public boolean isRedPath3(){
    return RobotContainer.auto_path_name.equals("AutoPath3");
  }

  public boolean isBluePath1(){
    return RobotContainer.auto_path_name.equals("AutoPath4");
  }

  public boolean isBluePath2(){
    return RobotContainer.auto_path_name.equals("AutoPath5");
  }

  public boolean isBluePath3(){
    return RobotContainer.auto_path_name.equals("AutoPath6");
  }
}
