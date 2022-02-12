package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoEater extends CommandBase{
    private final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    int count;
    boolean Gate1value;
    boolean Gate2value;
    boolean Gate3value;
    boolean intakevalue;
enum  MStates{
    DoNothing,
    WaitBall1,
    MoveMag1,
    BallPos1,
    WaitBall2,
    MoveMag2,
    FullMag,
    WaitShoot,
}
MStates stage;
public CargoEater(){
    this.intake = RobotContainer.RC().intake;
    this.magazine = RobotContainer.RC().magazine;
}
@Override
    public void initialize(){
        stage = MStates.DoNothing;
        count = 0;
        SmartDashboard.putString("Magazine_State", stage.toString());
        Gate1value = false;
        Gate2value = false;
        Gate3value = false;
        intakevalue = false;
    }
    
    @Override
    public void execute(){
        SmartDashboard.putString("Magazine_State", stage.toString());
        Gate1value = SmartDashboard.getBoolean("Gate1", false);
        Gate2value = SmartDashboard.getBoolean("Gate2", false);
        Gate3value = SmartDashboard.getBoolean("Gate3", false);
        intakevalue = SmartDashboard.getBoolean("IntakeValue", false);
        switch(stage){
            case DoNothing:
                if(intakevalue){
                    stage = MStates.WaitBall1;
                    magazine.driveWheelOn(0.001);
                }
            break; 
                
            case WaitBall1:
                if(Gate1value){
                    magazine.driveWheelOn(0.5);
                    stage = MStates.MoveMag1;
                }
            break;

            case MoveMag1:
                if(Gate2value){
                    stage = MStates.BallPos1;
                    magazine.driveWheelOff();
                }
            break;

            case BallPos1:
                stage = MStates.WaitBall2;
            break;

            case WaitBall2:
                if (Gate1value){
                    magazine.driveWheelOn(0.5);
                    stage = MStates.MoveMag2;
                }
            break;

            case MoveMag2:
                if(Gate1value){
                    magazine.driveWheelOff();
                    stage = MStates.FullMag;
                }
            break;

            case FullMag:
                if(Gate3value){
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
        magazine.driveWheelOff();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}

    


