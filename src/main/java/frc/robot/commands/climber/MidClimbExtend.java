package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class MidClimbExtend extends CommandBase {

    private final Climber climber;
    
    public MidClimbExtend(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }
    
    @Override
    public void initialize() {
        climber.setAmperageLimit(Constants.ClimbSettings.MAX_AMPERAGE);
    }

    @Override
    public void execute() {
        climber.setExtension(Constants.ClimbSettings.MID_EXTENSION_LENGTH);
        climber.setRotation(Constants.ClimbSettings.MID_EXTENSION_ROTATION);
    }

    @Override
    public boolean isFinished() {
        return climber.checkIsFinished(Constants.ClimbSettings.MID_EXTENSION_LENGTH, Constants.ClimbSettings.MID_EXTENSION_ROTATION);
    }


}
