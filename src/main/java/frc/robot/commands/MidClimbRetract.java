package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class MidClimbRetract extends CommandBase {
    private final Climber climber;

    public MidClimbRetract(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setExtension(0);
        climber.setRotation(0);
    }

    @Override
    public void execute() {
        // empty yet again
    }

    @Override
    // TODO: Velocity better than position because it allows for some error?
    public boolean isFinished() {
        return ((Math.abs(climber.getLeftEncoder().getPosition()) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
               && (Math.abs(climber.getRightEncoder().getPosition()) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
        // TODO: Add checks for rotation also
        );
    }
}
