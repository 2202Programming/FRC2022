package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class MidClimbExtend extends CommandBase {

    private final Climber climber;


    // Length to extend to reach mid bar
    private final double MID_EXTENSION_LENGTH = 60;

    // Degrees to rotate to reach mid bar
    private final double MID_EXTENSION_ROTATION = 0;
    
    public MidClimbExtend(Climber climber) {
        this.climber = climber;
    }
    
    @Override
    public void initialize() {
        climber.getLeftEncoder().setPosition(0);
        climber.getRightEncoder().setPosition(0);
        // TODO: Add resetting for rotation also
        climber.setExtension(MID_EXTENSION_LENGTH);
        climber.setRotation(MID_EXTENSION_ROTATION);
    }

    @Override
    public void execute() {
        // :)
    }

    @Override
    public boolean isFinished() {
        return(climber.getLeftEncoder().getPosition() >= MID_EXTENSION_LENGTH 
        && climber.getRightEncoder().getPosition() >= MID_EXTENSION_LENGTH
        // TODO: Check conditions for rotation motors also
        );
    }


}
