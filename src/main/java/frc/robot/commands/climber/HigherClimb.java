package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

public class HigherClimb extends SequentialCommandGroup {
    
    // dirve.toClimbStart
    // climbMid
    // stabilize etc.

    public HigherClimb(Climber climber) {
        super();
        this.addCommands(new HigherClimbExtend(climber), new HigherClimbRetract(climber));
    }

}
