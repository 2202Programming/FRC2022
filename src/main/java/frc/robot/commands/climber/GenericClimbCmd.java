package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

public class GenericClimbCmd extends CommandBase {

    private final Climber climber;

    // desired extension and rotation positions
    private double ext_pos;
    private double rot_pos;
    
    public GenericClimbCmd(Climber climber, double ext_pos, double rot_pos) {
        this.climber = climber;
        this.ext_pos = ext_pos;
        this.rot_pos = rot_pos;
    }
    
    @Override
    public void initialize() {
        climber.setAmperageLimit(Constants.ClimbSettings.MAX_AMPERAGE);
    }

    @Override
    public void execute() {
        climber.setExtension(ext_pos);
        climber.setRotation(rot_pos);
    }

    @Override
    public boolean isFinished() {
        return climber.checkIsFinished(ext_pos, rot_pos);
    }


}
