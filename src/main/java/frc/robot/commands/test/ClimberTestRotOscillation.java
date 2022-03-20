// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.MoveArmsTo;
import frc.robot.subsystems.climber.Climber;
//import static frc.robot.commands.climber.MidClimb.travers_start_ext;
import static frc.robot.commands.climber.MidClimb.*;


public class ClimberTestRotOscillation extends SequentialCommandGroup {
    
    // dirve.toClimbStart
    // climbMid
    // stabilize etc.
    double cycleCount = 0;
    double number = 0;
    Climber climber;

    public ClimberTestRotOscillation(Climber climber) {
        super();

        this.climber = climber;
        this.addCommands(new MoveArmsTo(climber, "cycle number " + number + " to -15 deg", 0, -15, true, true).withTimeout(1),
            new MoveArmsTo(climber, "cycle number " + number + " to 30 deg", 0, 30, true, true).withTimeout(1));
    }

    @Override
    public void execute() {
        if (cycleCount == 10) {
            number++;
            this.addCommands(new MoveArmsTo(climber, "cycle number " + number + " to -15 deg", 0, -15, true, true).withTimeout(1),
            new MoveArmsTo(climber, "cycle number " + number + " to 30 deg", 0, 30, true, true).withTimeout(1));
            cycleCount = 0;
        }
        cycleCount++;
    }

}
