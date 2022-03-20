package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

//import static frc.robot.Constants.ClimbSettings;
public class MidClimb extends SequentialCommandGroup {
    public static final double midext = 18;
    public static final double midrot = -9.5;
    public static final double partialpullupext = 0;

    public static final double pullupext = -2.70;
    public static final double pulluprotForward = 8;      //max forward on rotation
    public static final double travers_start_ext = 10;
    public static final double travers_start_rot = 15;

    /**
     * 
     * @param climber
     */

    public MidClimb(Climber climber) {
        super();

        double TO = 30.0; // default timeout for testing

        this.addCommands(
        new MoveArmsTo(climber, "extendForMid", midext, midrot/2, true, true),
        new MoveArmsTo(climber, "rotateForMidBar", midext, midrot, true, true).withTimeout(2), // timeout needed
        new MoveArmsTo(climber, "partial-pullup", partialpullupext, midrot, true, true).withTimeout(TO),
        new MoveArmsTo(climber, "pullupMid", pullupext, midrot, true, true).withTimeout(TO),
        new MoveArmsTo(climber, "rotateCG1", pullupext, pulluprotForward, true, true).withTimeout(TO),
        new MoveArmsTo(climber, "extend to hang", midext, partialpullupext, true, true).withTimeout(TO));
    }
}
