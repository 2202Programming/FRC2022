// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hid.SideboardController.SBButton;

public class auto_cmd extends SequentialCommandGroup {

  public auto_cmd() {
   
    addCommands(
      //run first path and shoot
      new InstantCommand(RobotContainer.RC().limelight::enableLED),
      new InstantCommand(RobotContainer.RC().drivetrain::printPose),
      new auto_cmd_group2(),
      new InstantCommand(RobotContainer.RC().drivetrain::printPose),

      //run 2nd path that correlates to 1st path if sideboard 4 is pressed
      new ConditionalCommand(new auto_cmd_terminal(), new WaitCommand(0), () -> RobotContainer.RC().driverControls.readSideboard(SBButton.Sw14)),
      new InstantCommand(RobotContainer.RC().drivetrain::printPose),
      new InstantCommand(()->{ RobotContainer.RC().sensors.setAutoEndPose(RobotContainer.RC().drivetrain.getPose()); })
    );
  }

}
