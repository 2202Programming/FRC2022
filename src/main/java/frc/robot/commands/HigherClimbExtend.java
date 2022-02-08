package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class HigherClimbExtend extends CommandBase {

    private final Climber climber;


    // degrees to rotate
    private final double HIGHER_EXTENSION_LENGTH = 30;
    private final double HIGHER_EXTENSION_ROTATION = 30;

    // toleranse --> TODO: Move to constants class and change to appropriate values
    private final double TOLERANCE_LENGTH = 0;
    private final double TOLERANCE_ROTATION = 0;

    // Length to extend to reach mid bar
    private final double MID_EXTENSION_LENGTH = 60;

    // Degrees to rotate to reach mid bar
    private final double MID_EXTENSION_ROTATION = 0;
    
    public HigherClimbExtend(Climber climber) {
        this.climber = climber;
    }
    
    @Override
    public void initialize() {
        climber.getLeftEncoder().setPosition(0);
        climber.getRightEncoder().setPosition(0);
        // TODO: Add resetting for rotation also
        climber.setExtension(HIGHER_EXTENSION_LENGTH);
        climber.setRotation(HIGHER_EXTENSION_ROTATION);
    }

    @Override
    public void execute() {
        // :)
    }

    @Override
    public boolean isFinished() {
        return(Math.abs(climber.getLeftEncoder().getPosition() - HIGHER_EXTENSION_LENGTH) <= TOLERANCE_LENGTH 
               && (Math.abs(climber.getRightEncoder().getPosition() - HIGHER_EXTENSION_LENGTH) <= TOLERANCE_LENGTH)
        // TODO: Check conditions for rotation motors also
        );
    }


}
