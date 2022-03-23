package frc.robot.commands.climber;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbSettings;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls.Id;

public class Climb extends SequentialCommandGroup {
    // define amount of time to wait for robot to stop swinging between steps - edit to tune
    private final int mid_stabilize_seconds = 3;
    private final int high_stabilize_seconds = 5;
    private Climber climber;

    private LinearFilter filter;

    public Climb(Climber climber, SwerveDrivetrain drivetrain) {
        super();
        this.climber = climber;
        // Use the following subcommands to climb climbMID climbHIGH stabilize
        this.addCommands(
            //TODO:  add drivetrain stuff
            new MidClimb(climber), 
            new WaitCommand(mid_stabilize_seconds),
            new TraverseClimb(climber),
            new WaitCommand(high_stabilize_seconds),
            new TraverseClimb(climber)
        );

        this.addRequirements(climber, drivetrain);
        filter = LinearFilter.movingAverage(3);

    }




    @Override
    public boolean isFinished() {
        var doKill = false;
        // if(RobotContainer.RC().driverControls.bind(Id.SwitchBoard, SBButton.Sw11).get()) {
        //     System.out.println("** CLIMBER KILLED reason=sb-button");
        //     doKill = true;
        // }

        if(RobotContainer.RC().driverControls.bind(Id.Driver, XboxButton.START).get()) {
            System.out.println("** CLIMBER KILLED reason=driver-button");
            doKill = true;
        }

        if(RobotContainer.RC().driverControls.bind(Id.Assistant, XboxButton.START).get()) {
            System.out.println("** CLIMBER KILLED reason=assistant-button");
            doKill = true;
        }

        var anglediff =filter.calculate(Math.abs(climber.getLeftRotation() - climber.getRightRotation()));
        if(anglediff > ClimbSettings.KILL_COUNT) {
            System.out.println("** CLIMBER KILLED reason=angle-diff anglediff=" + anglediff);
            doKill = true;
        }

        if(doKill) {
            climber.setRotSpeed(0);
            climber.setExtSpeed(0);
            return true;
        }

        return super.isFinished();
    }
}