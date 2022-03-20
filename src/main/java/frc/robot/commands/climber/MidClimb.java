package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;

//import static frc.robot.Constants.ClimbSettings;
public class MidClimb extends SequentialCommandGroup {
    public static final double midext = 16;
    public static final double midrot = -21;
    public static final double pullupext = -3;
    public static final double travers_start_ext = 15;
    public static final double travers_start_rot = -20;

    /**
     * 
     * @param climber
     */

    public MidClimb(Climber climber) {
        super();

        double TO = 3.0; // default timeout for testing

        this.addCommands(
                new MoveArmsTo(climber, midext, 0, true, true).withName("extendForMid"),
                new MoveArmsTo(climber, midext, midrot, true, true).withName("rotateForMidBar").withTimeout(2), // timeout needed
                new MoveArmsTo(climber, pullupext, midrot, true, true).withName("pullupMid").withTimeout(TO),
                new MoveArmsTo(climber, pullupext, midrot + 5, true, true).withName("rotateCG1").withTimeout(TO),
                new MoveArmsTo(climber, pullupext, 0, true, true).withName("rotateCG2").withTimeout(TO),

                new MoveArmsTo(climber, midext, 0, true, true).withName("extend to hang").withTimeout(TO));
    }
}
