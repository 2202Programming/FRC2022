package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class Stabilize extends CommandBase {
    
    private final Climber climber;
    private long startTime;

    // How long to sleep for to allow stabilization
    private final double MS_TO_WAIT;

    public Stabilize(Climber climber, long msToWait) {
        this.climber = climber;
        this.MS_TO_WAIT = msToWait;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        //use velocity mode and cmd of 0 to hold our positions
        climber.setExtSpeed(0, 0);
        climber.setRotSpeed(0, 0);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return ((System.currentTimeMillis() - startTime) < MS_TO_WAIT);
    }

}
