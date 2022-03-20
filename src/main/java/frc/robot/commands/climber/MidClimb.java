package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

//import static frc.robot.Constants.ClimbSettings;
public class MidClimb extends SequentialCommandGroup {
    public static final double midext = 16;
    public static final double midrot = -21;
    public static final double partialpullupext = 10;
    
    public static final double pullupext = -3;
    public static final double travers_start_ext = 10;
    public static final double travers_start_rot = 15;

    /**
     * 
     * @param climber
     */

    public MidClimb(Climber climber) {
        super();

        double TO = 3.0; // default timeout for testing

        this.addCommands(
                new MoveArmsTo(climber, "extendForMid", midext, 0, true, true),
                new MoveArmsTo(climber, "rotateForMidBar", midext, midrot, true, true).withTimeout(2), // timeout needed
                new MoveArmsTo(climber, "pullupMid", pullupext, midrot, true, true).withTimeout(TO),
                new MoveArmsTo(climber, "rotateCG1", pullupext, midrot + 5, true, true).withTimeout(TO),
                new MoveArmsTo(climber, "rotateCG2", pullupext, 0, true, true).withTimeout(TO),

                new MoveArmsTo(climber, "extend to hang", midext, -10, true, true).withTimeout(TO));
    }
}
