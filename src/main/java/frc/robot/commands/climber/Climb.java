package frc.robot.commands.climber;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Climb extends SequentialCommandGroup {
    // define amount of time to wait for robot to stop swinging between steps - edit to tune
    private final int mid_stabilize_seconds = 3;
    private final int high_stabilize_seconds = 5;

    public Climb(Climber climber, SwerveDrivetrain drivetrain) {

        super();
        // Use the following subcommands to climb climbMID climbHIGH stabilize
        // TODO: Implement all these commands
        this.addCommands(new MidClimb(climber), 
                         new Stabilize(climber, mid_stabilize_seconds * 1000), //stabilize expects delay in milliseconds
                         new HigherClimb(climber), 
                         new Stabilize(climber, high_stabilize_seconds * 1000), //stabilize expects delay in milliseconds
                         new HigherClimb(climber));

        this.addRequirements(climber, drivetrain);
    }
}