package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;

//import static frc.robot.Constants.ClimbSettings;
public class MidClimb extends SequentialCommandGroup {
    public static final double midext = 18;
    public static final double midreachrot = -15;
    public static final double midrot = -10;
    public static final double partialpullupext = 5;

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
        new MoveArmsTo(climber, "extendForMid", midext, midreachrot/2, true, true),
        new MoveArmsTo(climber, "rotateForMidBar", midext, midreachrot, true, true).withTimeout(2), // timeout needed
        new MoveArmsTo(climber, "partial-pullup1", partialpullupext, midreachrot, true, true).withTimeout(TO),
        new MoveArmsTo(climber, "partial-pullup2", partialpullupext, midrot, true, true).withTimeout(TO),
        new MoveArmsTo(climber, "pullupMid", pullupext, midrot, true, true).withTimeout(TO),
        new MoveArmsTo(climber, "rotateCG1", pullupext, pulluprotForward, true, true).withTimeout(TO),
        new MoveArmsTo(climber, "extend to trav", travers_start_ext, pulluprotForward, true, true).withTimeout(TO),
        new MoveArmsTo(climber, "rotate to trav", travers_start_ext, travers_start_rot, true, true).withTimeout(TO));
    }
}
