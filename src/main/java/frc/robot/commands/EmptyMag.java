package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;

public class EmptyMag extends CommandBase{
    private final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    
    public EmptyMag(){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        magazine.expellCargo(0.5); //filler constant
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    
    @Override
    public void end(boolean interrupted){
        
    }
}
