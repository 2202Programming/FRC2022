package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import static frc.robot.Constants.ClimbSettings;
public class MidClimb extends SequentialCommandGroup {
    
    // dirve.toClimbStart
    // climbMid
    // stabilize etc.

    public MidClimb(Climber climber) {
        super();
        this.addCommands(
            new MoveArmsTo(climber, ClimbSettings.MID_EXTENSION_LENGTH, ClimbSettings.MID_EXTENSION_ROTATION), 
            new WaitCommand(5.0),
            new MoveArmsTo(climber, 0.0 , 0.0)
     );
    }

}
