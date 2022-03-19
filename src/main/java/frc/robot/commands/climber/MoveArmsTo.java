package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class MoveArmsTo extends CommandBase {

    final Climber climber;
    //Positions we want to get to
    final double ext;   
    final double rot;
    final boolean syncArms;
    final boolean endOuterLoop;
    
    public MoveArmsTo(Climber climber, double ext_pos, double rot_pos, boolean syncArms, boolean endOuterLoop)  {
        this.climber = climber;
        this.ext = ext_pos;
        this.rot = rot_pos;
        this.syncArms = syncArms;
        this.endOuterLoop = endOuterLoop;
        addRequirements(climber);
    }

    public MoveArmsTo(Climber climber, double ext_pos, double rot_pos) {
         this(climber, ext_pos, rot_pos, false, true);
     }

    @Override
    public void initialize() {
        climber.setArmSync(syncArms);
        climber.setOuterLoop(true);
        climber.setExtension(ext);
        climber.setRotation(rot);
    }

    @Override
    public void end(boolean interrupted) {
        climber.hold();
        climber.setArmSync(syncArms);
        climber.setOuterLoop(endOuterLoop);
    }


    @Override
    public boolean isFinished() {
        return climber.outerLoopDone();
    }

}
