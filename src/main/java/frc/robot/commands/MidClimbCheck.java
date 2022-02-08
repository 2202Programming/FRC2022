package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

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
        climber.setExtension(0);
        climber.setRotation(0);
    }

    @Override
    public void execute() {
        // No need to change anything.
    }

    @Override
    public boolean isFinished() {
        // If climbers not moving then should be at extension 0 and rotation 0
        return ((climber.getLeftEncoder().getVelocity() == 0) && (climber.getRightEncoder().getVelocity() == 0))
            // TODO: check rotation motors also
            ;
    }

    //TODO: Possibly add rotation checks?

    
}
