package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class climberTest extends CommandBase {

    private Climber climber;
    NetworkTable table;
    
    //Magnitude of rotation / extension left / right motors
    NetworkTableEntry ntRot;
    NetworkTableEntry ntExt;
    
    public climberTest(Climber climber){
        this.climber = climber;
        addRequirements(climber);
        table = NetworkTableInstance.getDefault().getTable("Climber");
        ntRot = table.getEntry("Rotation");
        ntExt = table.getEntry("Extension");
        ntRot.setDouble(0);
        ntExt.setDouble(0);


    }

    @Override
    public void execute() {
        ntRot = table.getEntry("Rotation");
        ntExt = table.getEntry("Extension");
        climber.setExtension(ntExt.getDouble(0));
        climber.setRotation(ntRot.getDouble(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
    
}
