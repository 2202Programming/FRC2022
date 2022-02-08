package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class HigherClimbRetract extends CommandBase {
    private final Climber climber;

    // tolerances --> TODO: Move to Constants and change to appropriate values
    private final double TOLERANCE_LENGTH = 0;
    private final double TOLERANCE_ROTATION = 0;

    public HigherClimbRetract(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.getLeftEncoder().setPosition(0);
        climber.getRightEncoder().setPosition(0);
        climber.setRotation(0);
    }

    @Override
    public void execute() {
        // empty yet again
    }

    @Override
    // TODO: Velocity better than position because it allows for some error?
    public boolean isFinished() {
        return ((Math.abs(climber.getLeftEncoder().getPosition()) <= TOLERANCE_LENGTH)
               && (Math.abs(climber.getRightEncoder().getPosition()) <= TOLERANCE_LENGTH)
        // TODO: Add checks for rotation also
        );
    }
}
