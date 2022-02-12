package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class MidClimbExtend extends CommandBase {

    private final Climber climber;
    
    public MidClimbExtend(Climber climber) {
        this.climber = climber;
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climber.setExtension(Constants.ClimbSettings.MID_EXTENSION_LENGTH);
        climber.setRotation(Constants.ClimbSettings.MID_EXTENSION_ROTATION);
    }

    @Override
    public boolean isFinished() {
        return(Math.abs(climber.getLeftEncoder().getPosition() - Constants.ClimbSettings.MID_EXTENSION_LENGTH) <= Constants.ClimbSettings.TOLERANCE_LENGTH 
               && (Math.abs(climber.getRightEncoder().getPosition() - Constants.ClimbSettings.MID_EXTENSION_LENGTH) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
        // TODO: Check conditions for rotation motors also
        );
    }


}
