package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class climberTest extends CommandBase {

    private Climber climber;
    NetworkTable table;
    
    //Magnitude of rotation / extension left / right motors
    NetworkTableEntry ntRot;
    NetworkTableEntry ntExt;
    double last_rot;
    double last_ext;
    
    public climberTest(Climber climber){
        this.climber = climber;
        addRequirements(climber);
        table = NetworkTableInstance.getDefault().getTable("Climber");
        ntRot = table.getEntry("Rotation");
        ntExt = table.getEntry("Extension");
        ntRot.setDouble(0);
        ntExt.setDouble(0);
        last_rot = 0;
        last_ext = 0;

    }

    @Override
    public void execute() {
        if (last_ext != ntExt.getDouble(0)){
            climber.setExtension(ntExt.getDouble(0));
            last_ext = ntExt.getDouble(0);
        }
        if (last_rot != ntRot.getDouble(0)){
            climber.setRotation(ntRot.getDouble(0));
            last_rot = ntRot.getDouble(0);
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //climber.stop();
    }
    
}
