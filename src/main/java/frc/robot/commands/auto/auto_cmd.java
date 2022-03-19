// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.ifx.DriverControls;

public class auto_cmd extends SequentialCommandGroup {

  public auto_cmd() {
   
    addCommands(
      //run first path and shoot
      new InstantCommand(RobotContainer.RC().drivetrain::printPose),
      new auto_cmd_group2(),
      new InstantCommand(RobotContainer.RC().drivetrain::printPose),
      new ConditionalCommand(new auto_cmd_terminal(), new WaitCommand(0), () -> RobotContainer.RC().driverControls.readSideboard(SBButton.Sw14)),
      new InstantCommand(RobotContainer.RC().drivetrain::printPose)
      //new ConditionalCommand(new auto_pathPlanner_cmd(m_drivetrain, "AutoPath1"), new WaitCommand(0), this::isRedPath1),
      // if any 2nd row sideboards are pressed, run second path and shoot
      //new ConditionalCommand(new auto_cmd_terminal(), null, ()->(m_controls.readSideboard(SBButton.Sw21)||m_controls.readSideboard(SBButton.Sw22)||m_controls.readSideboard(SBButton.Sw23)))
    );
  }

}
