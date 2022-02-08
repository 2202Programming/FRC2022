package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class MidClimbRetract extends CommandBase {
    private final Climber climber;

    public MidClimbRetract(Climber climber) {
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
        return ((climber.getLeftEncoder().getPosition() == 0)
        && (climber.getRightEncoder().getPosition() == 0)
        // TODO: Add checks for rotation also
        );
    }
}
