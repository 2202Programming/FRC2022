package frc.robot.commands;

import org.opencv.highgui.HighGui;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
    private final Climber climber;
    private enum Level {MID, HIGH, TRAVERSE}
    private final Level level;

    // distances and stuff
    private final double MID_HEIGHT = 60.25;
    private final double MID_HIGH_DISTANCE = 24;
    private final double HIGH_HEIGHT = 75.625;
    private final double HIGH_TRAVERSE_DISTANCE = 24;
    private final double TRAVERSE_HEIGHT = 91;
    private boolean isFinished = false;

    public Climb(Climber climber, Level level) {
        this.climber = climber;
        this.level = level;
    }

    @Override
    public void initialize() {
        // Set velocity to 0
        climber.stop();
        // Reset to default positions (no extension, flat vertical)
        climber.setExtension(0);
        climber.setRotation(0);
    }


    @Override
    public void execute() {
        switch (level) {
            case MID:
                climber.setExtension(MID_HEIGHT);
                climber.setRotation(0);
            case HIGH:
                climber.setExtension(Math.sqrt(Math.pow(HIGH_HEIGHT - MID_HEIGHT, 2) + Math.pow(MID_HIGH_DISTANCE, 2)));
                climber.setRotation(90 - Math.atan((HIGH_HEIGHT - MID_HEIGHT) / MID_HIGH_DISTANCE));
            case TRAVERSE:
                climber.setExtension(Math.sqrt(Math.pow(TRAVERSE_HEIGHT - HIGH_HEIGHT, 2) + Math.pow(HIGH_TRAVERSE_DISTANCE, 2)));
                climber.setRotation(90 - Math.atan((TRAVERSE_HEIGHT - HIGH_HEIGHT) / HIGH_TRAVERSE_DISTANCE));
        }
        isFinished = true;
    }

    @Override
    public void end (boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }


    
}
