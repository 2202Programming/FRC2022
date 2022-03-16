package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

public class HigherClimbRetract extends CommandBase {
    private final Climber climber;

    public HigherClimbRetract(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setAmperageLimit(Constants.ClimbSettings.MAX_AMPERAGE);
    }

    @Override
    public void execute() {
        climber.setExtension(0);
        climber.setRotation(0);
    }

    @Override
    public boolean isFinished() {
        return climber.checkIsFinished(0, 0);
    }
}
