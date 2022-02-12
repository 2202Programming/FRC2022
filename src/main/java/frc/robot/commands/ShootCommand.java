package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;



public class ShootCommand extends CommandBase{ 
    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    int ballCount = 999;

    NetworkTable table;
    NetworkTableEntry ntUpperRPM;   //FW speeds (output)
    NetworkTableEntry ntLowerRPM;
    NetworkTableEntry ntBallVel;    // ball physics (input) 
    NetworkTableEntry ntBallRPS;
    
    ShooterSettings  cmdSS;         // instance the shooter sees
    ShooterSettings  prevSS;        // instance for prev State
    final ShooterSettings defaultShooterSettings = new ShooterSettings(15.0, 10.0, USE_CURRENT_ANGLE, 0.01);

    enum Stage{
        DoNothing,
        WaitingForFlyWheel,
        Shooting,
    }Stage stage;
    public ShootCommand(){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
    }

    @Override
    public void initialize(){
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");
        cmdSS = new ShooterSettings(); //defaultShooterSettings SUS 
        stage = Stage.DoNothing;
        ballCount = ballCount; //SUSpect to change
        shooter.off();
    }

    @Override
    public void execute(){
        switch(stage){
            case DoNothing:
                magazine.driveWheelOff();
                cmdSS = defaultShooterSettings;
                shooter.spinup(cmdSS);
                stage = Stage.WaitingForFlyWheel;
            break;

            case WaitingForFlyWheel:
                if(shooter.isReadyToShoot()){
                    magazine.driveWheelOn(0.99);
                    stage = Stage.Shooting;
                }
            break;

            case Shooting:
                if(!shooter.isReadyToShoot()){
                    magazine.driveWheelOff();
                    stage = Stage.WaitingForFlyWheel;
                }
            break;
        }
    }

    @Override
    public void end(boolean interrupted){
        stage = Stage.DoNothing;
        magazine.driveWheelOff();
        cmdSS = new ShooterSettings(); //defaultShooterSettings SUS 
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
