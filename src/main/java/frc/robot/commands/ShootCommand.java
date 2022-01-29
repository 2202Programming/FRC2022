package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;

public class ShootCommand extends CommandBase{
    final Magazine_Subsystem magazine;
    final Magazine_Subsystem intake;
    final Shooter_Subsystem shooter;
    int count;

    enum Stage{
        DoNothing,
        WaitingForSolution,
        BackingMagazine,
        WaitingForFlyWheel,
        Shooting,
    }Stage stage;
    public ShootCommand(){
        this.intake = RobotContainer.RC().intake;
    }

    @Override
    public void initialize(){
        stage = Stage.DoNothing;
        count = 0;
    }
    @Override
    public void execute(){
        switch(stage){
            case DoNothing:
                magazine.beltOn(0.001);
                stage = Stage.BackingMagazine;
            break;
            case BackingMagazine:
                if(count >= 2){
                    magazine.beltOff();
                    stage = Stage.WaitingForSolution;
                }
                count++;
            break;
            case WaitingForSolution:
                shooter.setSpeed(1.0);
                stage = Stage.WaitingForFlyWheel;
            break;
            case WaitingForFlyWheel:
                if(shooter.isReadyToShoot()){
                    magazine.beltOn(0.1);
                    stage = Stage.Shooting;
                }
            break;
            case Shooting:
                if(!shooter.isReadyToShoot()){
                    magazine.beltOff();
                    stage = Stage.WaitingForFlyWheel;
                    count--;
                }
            break;
        }
        count++;
    }
    @Override
    public void end(boolean interrupted){
        
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
