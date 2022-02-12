package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
// subsystems
import frc.robot.subsystems.Climber;

public class MidClimbCheck extends CommandBase {
    private final Climber climber;

    public MidClimbCheck(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        // Starting here so does not finish immediately.
        
    }

    @Override
    public void execute() {
        climber.setExtension(0);
        climber.setRotation(0);
    }

    @Override
    public boolean isFinished() {
        return ((Math.abs(climber.getLeftEncoder().getPosition()) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
               && (Math.abs(climber.getRightEncoder().getPosition()) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
        // TODO: Add checks for rotation also
        );
    }

    //TODO: Possibly add rotation checks?

    
}
