package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Positioner_Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;
import frc.robot.util.PoseMath;


public class VelShootCommand extends CommandBase{ 
    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final Positioner_Subsystem positioner;
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    final int BackupPeriod;

    int ballCount = 999;
    int backupCounter = 0;
    double currentDistance = 0;

    NetworkTable table;
    NetworkTableEntry ntUpperRPM;   //FW speeds (output)
    NetworkTableEntry ntLowerRPM;
    NetworkTableEntry ntBallVel;    // ball physics (input) 
    NetworkTableEntry shooterState;
    NetworkTableEntry distance;
    NetworkTableEntry NToutOfRange;
    public final String NT_Name = "Shooter"; 

    ShooterSettings specialSettings;
    
    ShooterSettings  cmdSS;         // instance the shooter sees
    ShooterSettings  prevSS;        // instance for prev State

    double calculatedVel = 20;

    private boolean finished = false;
    private boolean solution = true;
    private boolean shooterAngleLongRange;
    private boolean outOfRange = false;
    private boolean freeShootingMode = false;
    double log_counter = 0;

    final static ShooterSettings defaultShooterSettings = new ShooterSettings(20.0, 0.0, USE_CURRENT_ANGLE, 0.01);

    public enum Stage{
        DoNothing("Do Nothing"),
        WaitingForFlyWheel("Waiting for flywheel"),
        BackingMagazine("Backing  Mag"),
        PreparingToShoot("Preparing to Shoot"),
        WaitingForSolution("Waiting for Solution"),
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
        this.positioner = RobotContainer.RC().positioner;
        specialSettings = shooterSettings;
        BackupPeriod = backupFrameCount;  //number of frames to move mag back slowly 5-20
        addRequirements(magazine,shooter,positioner);

        table = NetworkTableInstance.getDefault().getTable(NT_Name);
        ntBallVel = table.getEntry("/BallVel");
        shooterState = table.getEntry("/ShooterState");
        distance = table.getEntry("/Distance");
        NToutOfRange = table.getEntry("/OutOfRange");
    }

    @Override
    public void initialize(){
        cmdSS = specialSettings; //defaultShooterSettings SUS 
        prevSS = new ShooterSettings(cmdSS);
        stage = Stage.DoNothing;
        shooter.off();
        magazine.driveWheelOff();
        shooterAngleLongRange = !positioner.isDeployed(); //Low shooting mode = long range = retracted
    }

    @Override
    public void execute(){
        NTupdates();
        calculateDistance();
        setPositioner();
        calculateVelocity();
        if(calculatedVel != cmdSS.vel){
            cmdSS = new ShooterSettings(calculatedVel, 0);
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
                if(!freeShootingMode) { //only check for a solution in guided shooting mode
                    if(solution){
                        stage = Stage.Shooting;
                        magazine.driveWheelOn(1.0);
                    }
                } else {
                    stage = Stage.Shooting;
                    magazine.driveWheelOn(1.0);
                }
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

    private void calculateDistance(){
        currentDistance = PoseMath.poseDistance(RobotContainer.RC().drivetrain.getPose(), Autonomous.hubPose);
    }

    //Low shooting mode = long range = retracted
    //min long range and max short range should not be equal to allow for some historesis to prevent rapid toggling at transition distance
    private void setPositioner(){
        shooterAngleLongRange = !positioner.isDeployed(); //check positioner angle from subsystem
        if ((currentDistance < Shooter.minLongRange) && shooterAngleLongRange) { //below long range, switch to short range
            positioner.deploy();
        } else if ((currentDistance > Shooter.maxShortRange) && !shooterAngleLongRange) { //above short trange, switch to long range
            positioner.retract();
        }
    
    }

    private void calculateVelocity(){       
        if (shooterAngleLongRange) {
            calculatedVel = 11.866 * Math.pow(Math.E, 0.1464*currentDistance); //distnce vs. velocity trendline for long range positioner
        } else {
            calculatedVel = 11.866 * Math.pow(Math.E, 0.1464*currentDistance); //distnce vs. velocity trendline for short range positioner
        }

        if (calculatedVel > Shooter.kMaxFPS){
            outOfRange = true;
            calculatedVel = Shooter.kMaxFPS; //don't ask shooter to go above max FPS otherwise can get stuck waiting for impossible goals
        } else {
            outOfRange = false;
        }
    }

    private void NTupdates(){
        log_counter++;
        if ((log_counter%20)==0) {
            ntBallVel.setDouble(calculatedVel);
            shooterState.setString(stage.toString());
            distance.setDouble(currentDistance);
            NToutOfRange.setBoolean(outOfRange);
        }
    }

    public boolean getSolution() {
        return this.solution;
    }

    public void setSolution(boolean solution) {
        this.solution = solution;
    }

    public void setFreeShootingMode(boolean freeShootingMode) {
        this.freeShootingMode = freeShootingMode;
    }
}
