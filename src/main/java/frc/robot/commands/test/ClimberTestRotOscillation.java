// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.MoveArmsTo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.ifx.DriverControls;


public class ClimberTestRotOscillation extends SequentialCommandGroup {
    
    // dirve.toClimbStart
    // climbMid
    // stabilize etc.
    Climber climber;
    static final double TO = 30;
    public ClimberTestRotOscillation(Climber climber, DriverControls dc) {
        super();

        this.climber = climber;
        this.addCommands(
            new MoveArmsTo(climber, "cycle number 1 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 1 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 2 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 2 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 3 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 3 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 4 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 4 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 5 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 5 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 6 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 6 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 7 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 7 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 8 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 8 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 9 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 9 to 30 deg", 0, 30, true, true, dc).withTimeout(TO),

            new MoveArmsTo(climber, "cycle number 10 to -15 deg", 0, -15, true, true, dc).withTimeout(TO),
            new MoveArmsTo(climber, "cycle number 10 to 30 deg", 0, 30, true, true, dc).withTimeout(TO)
        );
    }
}
