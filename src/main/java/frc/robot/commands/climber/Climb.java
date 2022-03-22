package frc.robot.commands.climber;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.ifx.DriverControls;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climb extends SequentialCommandGroup {
    // define amount of time to wait for robot to stop swinging between steps - edit to tune
    private final int mid_stabilize_seconds = 3;
    private final int high_stabilize_seconds = 5;

    public Climb(Climber climber, SwerveDrivetrain drivetrain, DriverControls dc) {
        super();
        // Use the following subcommands to climb climbMID climbHIGH stabilize
        this.addCommands(
            //TODO:  add drivetrain stuff
            new MidClimb(climber, dc), 
            new WaitCommand(mid_stabilize_seconds),
            new TraverseClimb(climber, dc),
            new WaitCommand(high_stabilize_seconds),
            new TraverseClimb(climber, dc)
        );

        this.addRequirements(climber, drivetrain);
    }
}