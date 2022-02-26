package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Autonomous;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;
import frc.robot.util.PoseMath;



public class VelShootCommand extends CommandBase{ 
    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    final int BackupPeriod;

    int ballCount = 999;
    int backupCounter = 0;

    NetworkTable table;
    NetworkTableEntry ntUpperRPM;   //FW speeds (output)
    NetworkTableEntry ntLowerRPM;
    NetworkTableEntry ntBallVel;    // ball physics (input) 
    NetworkTableEntry shooterState;

    ShooterSettings specialSettings;
    
    ShooterSettings  cmdSS;         // instance the shooter sees
    ShooterSettings  prevSS;        // instance for prev State

    double calculatedVel = 20;

    private boolean finished = false;

    final static ShooterSettings defaultShooterSettings = new ShooterSettings(20.0, 0.0, USE_CURRENT_ANGLE, 0.01);

    public enum Stage{
        DoNothing("Do Nothing"),
        WaitingForFlyWheel("Waiting for flywheel"),
        BackingMagazine("Backing  Mag"),
        PreparingToShoot("Preparing to Shoot"),
        WaitingForSolution("doing complex math"),
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
    
    public VelShootCommand(ShooterSettings shooterSettings, int backupFrameCount){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        specialSettings = shooterSettings;
        BackupPeriod = backupFrameCount;  //number of frames to move mag back slowly 5-20
        addRequirements(magazine,shooter);
    }

    @Override
    public void initialize(){
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");
        ntBallVel = table.getEntry("BallVel");
        shooterState = table.getEntry("ShooterState");

        cmdSS = specialSettings; //defaultShooterSettings SUS 
        prevSS = new ShooterSettings(cmdSS);
        stage = Stage.DoNothing;
        shooter.off();
        magazine.driveWheelOff();
    }

    @Override
    public void execute(){
        NTupdates();
        calculateVelocity();
        if(calculatedVel != cmdSS.vel){
            cmdSS = new ShooterSettings(calculatedVel, cmdSS.rps, cmdSS.angle, cmdSS.velTol);
            shooter.spinup(cmdSS);
        }

        switch(stage){
            case DoNothing:
                backupCounter = 0;
                stage = Stage.BackingMagazine;
                magazine.expellCargo(0.1);
            break;

            case BackingMagazine:                
                backupCounter++;
                if (backupCounter > BackupPeriod) {
                    backupCounter = 0;
                    magazine.driveWheelOff();
                    stage = Stage.WaitingForFlyWheel;
                    shooter.spinup(cmdSS);
                }                
            break;

            case WaitingForFlyWheel:
                if(shooter.isReadyToShoot()){
                    stage = Stage.WaitingForSolution;
                }
            break;

            case WaitingForSolution:
                // do fancy check and when ready, goto shooting
                stage = Stage.Shooting;
                magazine.driveWheelOn(1.0);
                break;

            case Shooting:
                if(!shooter.isReadyToShoot()){
                    magazine.driveWheelOff();
                    stage = Stage.WaitingForFlyWheel;
                }
            break;
            default:
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

    private void calculateVelocity(){
        double currentDistance = PoseMath.poseDistance(RobotContainer.RC().drivetrain.getPose(), Autonomous.hubPose);
        calculatedVel = 10.545 * Math.pow(Math.E, 0.446*currentDistance); //distnce vs. velocity trendline is y = 10.545e0.0446x
    }

    private void NTupdates(){
        ntBallVel.setDouble(calculatedVel);
        shooterState.setString(stage.toString());    }
}
