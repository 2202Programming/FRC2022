package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.MagazineSettings;
import frc.robot.commands.Shoot.BasicShootCommand.Stage;

        //TODO
        //Driver input to shoot ball, we want to ready ball for shooting
        //Intake has ball, we want to take and keep ball in the magazine
        //Expell ball from magazine, we want to remove ball from driver input

public class MagazineGatedCommand extends MagazineCommand {

    int ball_count;
    public enum MagazineState {
        Empty,
        OneBallPos1,
        OneBallPos2,
        TwoBalls,
    }

    MagazineState state = MagazineState.Empty;

    //Constructor
    public MagazineGatedCommand(DoubleSupplier magazineSpeedFunction, MagazineMode mode){
        super(magazineSpeedFunction, mode);
    
    }
    
    public MagazineGatedCommand(MagazineMode mode){
        this(()->MagazineSettings.defaultMagazineSpeed, mode);
    }

    
    
    public void initialize() {
        super.initialize();
        //sort out ligth gates and ball positions for state machine
        state = (magazine.lowerGateBlocked()) ? MagazineState.OneBallPos1 : MagazineState.Empty;
        if (magazine.upperGateBlocked()) {
            state = (state == MagazineState.Empty)  ? MagazineState.OneBallPos2 : MagazineState.TwoBalls;
        }
       
    }

    public void execute() {
        // //TODO more variables to consider
        // if(mode == MagazineMode.LoadCargo){
        //     //We will take cargo from the intake
        //     magazine.driveWheelOn(magazineSpeed.getAsDouble());
        // }else if(mode == MagazineMode.ExpellCargo){
        //     //We will expell cargo from the magazine
        //     magazine.driveWheelOn(-magazineSpeed.getAsDouble());
        // }

       

    }

    public void setFinished(){
        finished = true;
    }
    
    public boolean isFinished() {
        return finished;
    }
    @Override
    public void end(boolean interrupted) {
        magazine.driveWheelOff();
    }

    void updateNT() {

    }

}
