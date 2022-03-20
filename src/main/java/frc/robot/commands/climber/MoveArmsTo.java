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
    final String name;
    
    public MoveArmsTo(Climber climber, double ext_pos, double rot_pos, boolean syncArms, boolean endOuterLoop)  {
        this.climber = climber;
        this.name = null;
        this.ext = ext_pos;
        this.rot = rot_pos;
        this.syncArms = syncArms;
        this.endOuterLoop = endOuterLoop;
        addRequirements(climber);
    }

    public MoveArmsTo(Climber climber, String name, double ext_pos, double rot_pos, boolean syncArms, boolean endOuterLoop)  {
        this.climber = climber;
        this.name = name;
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
        System.out.println("****" + name + " Arms moving to (" + ext +"," +rot+") *********");
    }

    @Override
    public void end(boolean interrupted) {
        climber.hold();
        climber.setArmSync(syncArms);
        climber.setOuterLoop(endOuterLoop);
        if (!interrupted)
            System.out.println("****" + name + " Completed****");
        else 
          System.out.println("****" + name + " Interrupted****");
    }


    @Override
    public boolean isFinished() {
        return climber.outerLoopDone();
    }

}
