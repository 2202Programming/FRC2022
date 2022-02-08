package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class MidClimbStabilize extends CommandBase {
    
    private final Climber climber;


    private long startTime;

    // How long to sleep for to allow stabilization
    private final double MS_TO_WAIT = 3000;

    public MidClimbStabilize(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        climber.setSpeed(0, 0);
        // TODO: Add something siilar for rotations
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return ((System.currentTimeMillis() - startTime) < MS_TO_WAIT);
    }

}
