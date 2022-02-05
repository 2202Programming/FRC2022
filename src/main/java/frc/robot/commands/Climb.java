package frc.robot.commands;

import org.opencv.highgui.HighGui;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
    private final Climber climber;

    // distances and stuff
    private final double MID_HEIGHT = 60.25;
    private final double MID_HIGH_DISTANCE = 24;
    private final double HIGH_HEIGHT = 75.625;
    private final double HIGH_TRAVERSE_DISTANCE = 24;
    private final double TRAVERSE_HEIGHT = 91;


    // variables used in execution
    private boolean isFinished = false;
    private boolean finishedClimbing = false;
    double currentCount = 0;

    private enum Stage {
        MID, HIGH, TRAVERSE
    }

    private Stage stage;


    
    public Climb(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        // Set velocity to 0
        while ((climber.getLeftEncoder().getVelocity() != 0) && (climber.getRightEncoder().getVelocity() != 0)) {
            climber.stop();
        }
        // Reset to default positions (no extension, flat vertical)
        while ((climber.getLeftEncoder().getPosition() != 0) && climber.getRightEncoder().getPosition() != 0) {
            climber.setExtension(0);
        }
        // TODO: Add while loop to check rotation
        climber.setRotation(0);

        stage = Stage.MID;
        currentCount = (climber.getLeftEncoder().getPosition() + climber.getRightEncoder().getPosition()) / 2;
    }

    @Override
    public void execute() {
        if (isFinished == true) {
            currentCount = (climber.getLeftEncoder().getPosition() + climber.getRightEncoder().getPosition()) / 2;
            isFinished = false;
            switch (stage) {
                case MID:
                    stage = Stage.HIGH;
                case HIGH:
                    stage = Stage.TRAVERSE;
                case TRAVERSE:
                    finishedClimbing = true;
            }
        }
        if (stage == Stage.MID) {
            while (currentCount < currentCount + MID_HEIGHT) {
                climber.setExtension(MID_HEIGHT);
            }
            isFinished = true;
        } else if (stage == Stage.HIGH) {
            // Get to high
            // TODO: Add while loop to check for current / targeted rotation
            climber.setRotation(90 - Math.atan((HIGH_HEIGHT - MID_HEIGHT) / MID_HIGH_DISTANCE));
            while (currentCount < currentCount
                    + Math.sqrt(Math.pow(HIGH_HEIGHT - MID_HEIGHT, 2) + Math.pow(MID_HIGH_DISTANCE, 2))) {
                climber.setExtension(Math.sqrt(Math.pow(HIGH_HEIGHT - MID_HEIGHT, 2) + Math.pow(MID_HIGH_DISTANCE, 2)));
            }
            isFinished = true;
        } else if (stage == Stage.TRAVERSE) {
            // Get to traverse
            currentCount = (climber.getLeftEncoder().getPosition() + climber.getRightEncoder().getPosition()) / 2;
            // TODO: Add while loop to check for current / targeted rotation
            climber.setRotation(90 - Math.atan((TRAVERSE_HEIGHT - HIGH_HEIGHT) / HIGH_TRAVERSE_DISTANCE));
            while (currentCount < currentCount
                    + Math.sqrt(Math.pow(TRAVERSE_HEIGHT - HIGH_HEIGHT, 2) + Math.pow(HIGH_TRAVERSE_DISTANCE, 2))) {
                climber.setExtension(
                        Math.sqrt(Math.pow(TRAVERSE_HEIGHT - HIGH_HEIGHT, 2) + Math.pow(HIGH_TRAVERSE_DISTANCE, 2)));
            }
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return finishedClimbing;
    }

}
