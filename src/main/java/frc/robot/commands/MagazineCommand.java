package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Magazine_Subsystem; 
import frc.robot.subsystems.ifx.DriverControls;

public class MagazineCommand extends CommandBase {
    private final Magazine_Subsystem magazine;
    private final DriverControls dc;
    public MagazineCommand(DriverControls dc){
        this.magazine = RobotContainer.RC().magazine;   // just get the magazine from RC
        this.dc = dc;                   // TODO: likely won't need DC, buttons will get bound to command

        //TODO:  What is this command going to do?  It's name should be a VERB - commands do...
    }
    
    // TODO: think about what if anything is needed for the commands life-cycle
    
    public void initialize() { 
        //TODO do something really useful here please.
        dc.getRotation(); 
        magazine.beltOff();
    }
    public void execute() {}
    public boolean isFinished() {return false;}
}
