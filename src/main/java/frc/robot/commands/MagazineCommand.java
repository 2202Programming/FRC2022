package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.BasicShootCommand.Stage;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;

        //TODO
        //Driver input to shoot ball, we want to ready ball for shooting
        //Intake has ball, we want to take and keep ball in the magazine
        //Expell ball from magazine, we want to remove ball from driver input

public class MagazineCommand extends CommandBase {
    //Defintions
    private final Magazine_Subsystem magazine;
    DoubleSupplier magazineSpeed;

    final Intake_Subsystem intake;
    //MagazineMode to either Load or Expell cargo on driver input
    public enum MagazineMode {
        LoadCargo, ExpellCargo
    }
    Stage stage;
    MagazineMode mode;

    //Constructor
    public MagazineCommand(DoubleSupplier magazineSpeedFunction, MagazineMode mode){
        this.magazine = RobotContainer.RC().magazine;   // just get the magazine from RC
        this.intake = RobotContainer.RC().intake;
        this.mode = mode;
        this.magazineSpeed = magazineSpeedFunction;

        addRequirements(magazine);
    }
    
    // TODO: think about what if anything is needed for the commands life-cycle
    
    public void initialize() {
        stage = Stage.DoNothing; 
        //TODO do something really useful here please.
        magazine.driveWheelOff();
        intake.off();
    }

    public void execute() {
        //TODO more variables to consider
        if(mode == MagazineMode.LoadCargo){
            //We will take cargo from the intake
            magazine.driveWheelOn(magazineSpeed.getAsDouble());
        }else if(mode == MagazineMode.ExpellCargo){
            //We will expell cargo from the magazine
            magazine.driveWheelOn(-magazineSpeed.getAsDouble());
        }
    }

    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        magazine.driveWheelOff();
    }

}
