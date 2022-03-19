package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
//import static frc.robot.Constants.ClimbSettings;
public class MidClimb extends SequentialCommandGroup {
    
    // dirve.toClimbStart
    // climbMid
    // stabilize etc.

    public MidClimb(Climber climber) {
        super();
        double midext = 16;
        double midrot = -21;
        double pullupext = -2;
        double travers_start_ext = 15;
        double travers_start_rot = -20;

        double TO = 3.0;  //default timeout for testing

        this.addCommands(
            new MoveArmsTo(climber, midext, 0, true, true).withName("extendForMid"),
            new MoveArmsTo(climber, midext, midrot, true, true).withName("rotateForMidBar").withTimeout(2),  //timeout needed
            new MoveArmsTo(climber, pullupext, midrot, true, true).withName("pullupMid").withTimeout(TO),
            new MoveArmsTo(climber, pullupext, -5, true, true).withName("rotateCG1").withTimeout(TO),
            new MoveArmsTo(climber, pullupext, 0, true, true).withName("rotateCG2").withTimeout(TO),
            
            new MoveArmsTo(climber, 15, 0, true, true).withName("extend to hang").withTimeout(TO),
            new WaitCommand(10).withName("hanging"),

            //goto Travers start
            new MoveArmsTo(climber, travers_start_ext , travers_start_rot, true, true).withName("traverseStart").withTimeout(3)
     );
    }

}
