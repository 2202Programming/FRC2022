package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

public class HigherClimbExtend extends CommandBase {

    private final Climber climber;
    
    public HigherClimbExtend(Climber climber) {
        this.climber = climber;
    }
    
    @Override
    public void initialize() {
        climber.setAmperageLimit(Constants.ClimbSettings.MAX_AMPERAGE);
    }

    @Override
    public void execute() {
        climber.setExtension(Constants.ClimbSettings.HIGHER_EXTENSION_LENGTH);
        climber.setRotation(Constants.ClimbSettings.HIGHER_EXTENSION_ROTATION);
    }

    @Override
    public boolean isFinished() {
        return climber.checkIsFinished(Constants.ClimbSettings.HIGHER_EXTENSION_LENGTH, Constants.ClimbSettings.HIGHER_EXTENSION_ROTATION);

    }


}
