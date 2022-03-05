package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class MidClimbDrive extends CommandBase {
    

    // TODO: I really don't want to deal with rotation stuff
    private final SwerveDrivetrain dt;

    public MidClimbDrive(SwerveDrivetrain dt) {
        this.dt = dt;
    }
    
    @Override
    public void initialize() {
        dt.drive(1, 1, 0);
    }
    
    @Override
    public void execute() {
        // No need to do anything
    }

    @Override
    public boolean isFinished() {
        return false;
        // color sensor detects white
    }
}
