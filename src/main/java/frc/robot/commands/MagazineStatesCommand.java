package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;


public class MagazineStatesCommand extends CommandBase{
    private final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    int count;

enum  MStates{
    DoNothing,
    WaitBall1,
    MoveBall1,
    BallPos1,
    WaitBall2,
    MoveBall2,
    FullMag,
    WaitShoot,
}
MStates stage;
public MagazineStatesCommand(){
    this.intake = RobotContainer.RC().intake;
    this.shooter = RobotContainer.RC().shooter;
    this.magazine = RobotContainer.RC().magazine;
}
@Override
    public void initialize(){
        stage = MStates.DoNothing;
        count = 0;
    }
    @Override
    public void execute(){
        switch(stage){
            case DoNothing:
                if(intake.isCargoDetected()){
                    stage = MStates.WaitBall1;
                }
            break;

            case WaitBall1:
                if(magazine.isGate1Blocked()){
                    stage = MStates.MoveBall1;
                }
            break;

            case MoveBall1:
                if(magazine.isGate2Blocked()){
                    stage = MStates.BallPos1;
                }
            break;

            case BallPos1:
                stage = MStates.WaitBall2;
            break;

            case WaitBall2:
                stage = MStates.MoveBall2;
            break;

            case MoveBall2:
                if(magazine.isGate1Blocked()){
                    stage = MStates.FullMag;
                }
            break;

            case FullMag:
                if(magazine.isGate3Blocked()){
                    stage = MStates.WaitShoot;
                }
            break;
            
            case WaitShoot:
            break;
        }
    count++;
    }
    
    @Override
    public void end(boolean interrupted){
        stage = MStates.DoNothing;
        magazine.beltOff();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}

    


