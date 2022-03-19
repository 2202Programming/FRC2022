package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

public class MidClimbExtend extends CommandBase {

    private final Climber climber;
    //Positions we want to get to
    double ext;   
    double rot;
    
    public MidClimbExtend(Climber climber, double ext_pos, double rot_pos) {
        this.climber = climber;
        this.ext = ext_pos;
        this.rot = rot_pos;
        addRequirements(climber);
    }
    
    @Override
    public void initialize() {
        climber.setArmSync(true);
        climber.setAmperageLimit(Constants.ClimbSettings.MAX_AMPERAGE);
        climber.setExtension(ext);
        climber.setRotation(rot);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setArmSync(false);
    }


    @Override
    public boolean isFinished() {
        return climber.checkIsFinished(ext, rot);
    }

}
