package frc.robot.commands.climber;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbSettings;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.ifx.DriverControls.Id;

//import static frc.robot.Constants.ClimbSettings;
public class MidClimb extends SequentialCommandGroup {
    public static final double midext = 18;
    public static final double midreachrot = -15;
    public static final double midrot = -10;
    public static final double partialpullupext = 5;

    public static final double pullupext = -2.70;
    public static final double pulluprotForward = 8; // max forward on rotation
    public static final double travers_start_ext = 10;
    public static final double travers_start_rot = 15;

    private Climber climber;

    private LinearFilter filter;

    /**
     * 
     * @param climber
     */

    public MidClimb(Climber climber) {
        super();

        double TO = 30.0; // default timeout for testing

        this.addCommands(new MoveArmsTo(climber, "extendForMid", midext, midreachrot / 2, true, true),
                new MoveArmsTo(climber, "rotateForMidBar", midext, midreachrot, true, true).withTimeout(2), // timeout
                                                                                                            // needed
                // new MoveArmsTo(climber, "partial-pullup1", partialpullupext, midreachrot,
                // true, true).withTimeout(TO),
                // new MoveArmsTo(climber, "partial-pullup2", partialpullupext, midrot, true,
                // true).withTimeout(TO),
                new MoveArmsTo(climber, "pullupMid", pullupext, midrot, true, true).withTimeout(TO),
                new MoveArmsTo(climber, "rotateCG1", pullupext, pulluprotForward, true, true).withTimeout(TO));

        this.climber = climber;

        filter = LinearFilter.movingAverage(3);
    }

    @Override
    public boolean isFinished() {
        var doKill = false;
        if (RobotContainer.RC().driverControls.bind(Id.SwitchBoard, SBButton.Sw11).get()) {
            System.out.println("** CLIMBER KILLED reason=sb-button");
            doKill = true;
        }

        if (RobotContainer.RC().driverControls.bind(Id.Driver, XboxButton.START).get()) {
            System.out.println("** CLIMBER KILLED reason=driver-button");
            doKill = true;
        }

        if (RobotContainer.RC().driverControls.bind(Id.Assistant, XboxButton.START).get()) {
            System.out.println("** CLIMBER KILLED reason=assistant-button");
            doKill = true;
        }

        var anglediff = filter.calculate(Math.abs(climber.getLeftRotation() - climber.getRightRotation()));
        if (anglediff > ClimbSettings.KILL_COUNT) {
            System.out.println("** CLIMBER KILLED reason=angle-diff anglediff=" + anglediff);
            doKill = true;
        }

        if (doKill) {
            climber.setRotSpeed(0);
            climber.setExtSpeed(0);
            return true;
        }

        return super.isFinished();
    }
}
