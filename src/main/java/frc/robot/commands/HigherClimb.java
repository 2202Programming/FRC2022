package frc.robot.commands;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class HigherClimb extends SequentialCommandGroup {
    
    // dirve.toClimbStart
    // climbMid
    // stabilize etc.

    public HigherClimb(Climber climber) {
        super(new HigherClimbExtend(climber), new HigherClimbRetract(climber));
    }

}
