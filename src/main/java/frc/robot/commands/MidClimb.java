package frc.robot.commands;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MidClimb extends SequentialCommandGroup {
    
    // dirve.toClimbStart
    // climbMid
    // stabilize etc.

    public MidClimb(Climber climber) {
        super();
        this.addCommands(new MidClimbExtend(climber), new MidClimbRetract(climber));
    }

}
