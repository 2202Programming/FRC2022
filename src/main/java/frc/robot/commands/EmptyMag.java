package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;

public class EmptyMag extends CommandBase{
    private final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    double expelRate;
    

    public EmptyMag() {
        this(.5);
    }
    public EmptyMag(double expelRate){
        this.intake = RobotContainer.RC().intake;
        this.magazine = RobotContainer.RC().magazine;
        this.expelRate = expelRate;
    }
    @Override
    public void initialize(){
        magazine.expellCargo(expelRate); //turns motor on
    }
    @Override
    public void execute(){        
    }

    @Override
    public boolean isFinished(){
        // Keep doing this until driver releases the button
        // and the command is unscheduled.
        return false;
    }
    
    @Override
    public void end(boolean interrupted){
        magazine.beltOff();
    }
}
