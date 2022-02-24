package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;



public class BasicShootCommand extends CommandBase{ 
    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    int ballCount = 999;
    int backupCounter = 0;

    NetworkTable table;
    NetworkTableEntry ntUpperRPM;   //FW speeds (output)
    NetworkTableEntry ntLowerRPM;
    NetworkTableEntry ntBallVel;    // ball physics (input) 
    NetworkTableEntry ntBallRPS;
    NetworkTableEntry shooterState;
    
    ShooterSettings  cmdSS;         // instance the shooter sees
    ShooterSettings  prevSS;        // instance for prev State

    private boolean finished = false;

    final ShooterSettings defaultShooterSettings = new ShooterSettings(20.0, 0.0, USE_CURRENT_ANGLE, 0.01);

    public enum Stage{
        DoNothing("Do Nothing"),
        WaitingForFlyWheel("Waiting for flywheel"),
        PreparingToShoot("Preparing to Shoot"),
        Shooting("Shooting");

        String name;

        private Stage(String name){
            this.name = name;
        }

        public String toString(){
            return name;
        }
    }
    
    Stage stage;
    
    public BasicShootCommand(){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
    }

    @Override
    public void initialize(){
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");
        ntBallVel = table.getEntry("BallVel");
        ntBallRPS = table.getEntry("BallRPS");
        shooterState = table.getEntry("ShooterState");

        ntBallVel.setDouble(0);
        ntBallRPS.setDouble(0);

        cmdSS = new ShooterSettings(); //defaultShooterSettings SUS 
        prevSS = new ShooterSettings(cmdSS);

        stage = Stage.DoNothing;
        shooter.off();
    }

    @Override
    public void execute(){
        shooterState.setString(stage.toString());

        switch(stage){
            case DoNothing:
                magazine.driveWheelOff();
                cmdSS = defaultShooterSettings;
                shooter.spinup(cmdSS);
                stage = Stage.WaitingForFlyWheel;
            break;

            case WaitingForFlyWheel:
                if(shooter.isReadyToShoot()){
                    magazine.driveWheelOff(); //dont advance indexer if shooting wheels aren't ready
                    stage = Stage.PreparingToShoot;
                }
            break;

            //back the balls away from wheels a touch
            case PreparingToShoot:
                magazine.expellCargo(-0.1);
                backupCounter++;
                if (backupCounter > 20) {
                    backupCounter = 0;
                    stage = Stage.Shooting;
                }                

            case Shooting:
                if(!shooter.isReadyToShoot()){
                    stage = Stage.WaitingForFlyWheel;
                } else magazine.driveWheelOn(1.0);
            break;
        }
    }

    @Override
    public void end(boolean interrupted){
        stage = Stage.DoNothing;
        magazine.driveWheelOff();
        shooter.off();
    }

    public void setFinished(){
        finished = true;
    }
    
    @Override
    public boolean isFinished(){
        return finished;
    }
}