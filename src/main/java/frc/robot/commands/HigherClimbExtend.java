package frc.robot.commands;

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
        // TODO: Add resetting for rotation also
        
    }

    @Override
    public void execute() {
        climber.setExtension(Constants.ClimbSettings.HIGHER_EXTENSION_LENGTH);
        climber.setRotation(Constants.ClimbSettings.HIGHER_EXTENSION_ROTATION);
    }

    @Override
    public boolean isFinished() {
        return(Math.abs(climber.getLeftEncoder().getPosition() - Constants.ClimbSettings.HIGHER_EXTENSION_LENGTH) <= Constants.ClimbSettings.TOLERANCE_LENGTH 
               && (Math.abs(climber.getRightEncoder().getPosition() - Constants.ClimbSettings.HIGHER_EXTENSION_LENGTH) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
        // TODO: Check conditions for rotation motors also
        );
    }


}
