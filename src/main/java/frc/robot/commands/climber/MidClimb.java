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
        double midext = 15;
        double midrot = -25;
        double pullupext = -2;
        this.addCommands(
            new MoveArmsTo(climber, midext, 0, true, true),
            new MoveArmsTo(climber, midext, midrot, true, true).withTimeout(5),
            new MoveArmsTo(climber, pullupext, midrot, true, true),
            new MoveArmsTo(climber, pullupext, 10, true, true),
            new MoveArmsTo(climber, pullupext, 0, true, true),
            
            new MoveArmsTo(climber, 15, 0, true, true),
            new WaitCommand(60),
            new MoveArmsTo(climber, 0.0 , 0.0, true, true)
     );
    }

}
