package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Shooter;
import frc.robot.commands.MagazineController;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Positioner_Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;
import frc.robot.util.PoseMath;


public class VelShootGatedCommand extends CommandBase implements SolutionProvider{ 
   
    public static final double USE_CURRENT_ANGLE = 0.0;

    final MagazineController mag_ctrl;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final Positioner_Subsystem positioner;
    final SolutionProvider solutionProvider;  
  
  
    
    final NetworkTableEntry nte_BallVel;    // ball physics (input) 
    final NetworkTableEntry nte_shooterState;
    final NetworkTableEntry nte_distance;
    final NetworkTableEntry nte_outOfRange;
    public final String NT_Name = "Shooter"; 

    ShooterSettings  m_shooterSettings;  // passed in setting, likely fixed
    ShooterSettings  cmdSS;              // calculated by this command

    //solution provider calculated values
    double calculatedVel = 20;
    double currentDistance = 0;

    private boolean finished = false;
    private boolean shooterAngleLongRange;
    private boolean outOfRange = false;
    private boolean autoVelocity = true;

    double log_counter = 0;

    final static ShooterSettings defaultShooterSettings = new ShooterSettings(20.0, 0.0, USE_CURRENT_ANGLE, 0.01);

    public enum Stage{
        WaitingForFlyWheel("Waiting for flywheel"),
        WaitingOnMag("MagCtrl Not ready"),
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
    
    /**
     * 
     * @param shooterSettings       container for shooter speeds/angle
     * @param mag_ctrl              interface for controlling magazine
     * @param solutionProvider      interface for target tracking 
     */

    public VelShootGatedCommand(ShooterSettings shooterSettings, MagazineController mag_ctrl, SolutionProvider solutionProvider){
        this.mag_ctrl = mag_ctrl;
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
    
        this.positioner = RobotContainer.RC().positioner;
        // the default solution provider is always true
        this.solutionProvider = (solutionProvider == null) ? this : solutionProvider;
        m_shooterSettings = shooterSettings;
        
        addRequirements(shooter, positioner);

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_Name);
        nte_BallVel = table.getEntry("/VelShootCmd/BallVel");
        nte_shooterState = table.getEntry("/VelShootCmd/ShooterState");
        nte_distance = table.getEntry("/VelShootCmd/Distance");
        nte_outOfRange = table.getEntry("/VelShootCmd/OutOfRange");
    }

    public VelShootGatedCommand(ShooterSettings shooterSettings, MagazineController mag_ctrl)
    {
        this(shooterSettings, mag_ctrl, null);
    }

    @Override
    public void initialize(){
        cmdSS = new ShooterSettings(m_shooterSettings); 
        stage = Stage.WaitingOnMag;
        shooter.off();
        shooterAngleLongRange = !positioner.isDeployed(); //Low shooting mode = long range = retracted
    }

    @Override
    public void execute(){
        calculateDistance();
        calculateVelocity();
        
        //if autovelocity is true will calculate a new RPM speed based on the distance and adjust positioner
        //otherwise RPMs should be constant based on the constructor parameters
        if (autoVelocity) {
            setPositioner();
            if(calculatedVel != cmdSS.vel){
                cmdSS.vel = calculatedVel;   //shouldn't have to create new object, just change vel
                shooter.spinup(cmdSS);
            }
        } 

        switch(stage){
            case WaitingOnMag:                
                if (mag_ctrl.safeToSpinUp()) {
                    stage = Stage.WaitingForFlyWheel;
                    intake.off();
                    shooter.spinup(cmdSS);              // spin shooter up
                }                
            break;

            case WaitingForFlyWheel:
                if (shooter.isReadyToShoot()) {
                    stage = Stage.WaitingForSolution;
                }
            break;

            case WaitingForSolution:
                if (solutionProvider.isOnTarget()) {
                    stage = Stage.Shooting;
                    mag_ctrl.feederOn();
                    intake.on(0.0, 0.2);
                }
                break;

            case Shooting:
                if (!shooter.isReadyToShoot()){
                    mag_ctrl.feederOff();
                    intake.off();
                    shooter.spinup(cmdSS); //in case a new velocity has been set due to a new distance
                    stage = Stage.WaitingForFlyWheel;
                }
            break;
            default:
                break;
        }

        NTupdates();
    }

    @Override
    public void end(boolean interrupted){
        mag_ctrl.feederOff();
        intake.off();
        shooter.off();
    }

    public void setFinished(){
        finished = true;
    }
    
    /*
    private double getManualVelocity(){
        double velocity = Shooter.mediumVelocity;
        if(RobotContainer.RC().driverControls.readSideboard(SBButton.Sw21)) velocity = Shooter.shortVelocity;
        else if(RobotContainer.RC().driverControls.readSideboard(SBButton.Sw23)) velocity = Shooter.longVelocity;
        return velocity;
    }
    */

    @Override
    public boolean isFinished(){
        return finished;
    }

    private void calculateDistance(){
        currentDistance = PoseMath.poseDistance(RobotContainer.RC().drivetrain.getPose(), Autonomous.hubPose);
        if (RobotContainer.RC().limelight.getTarget() && RobotContainer.RC().limelight.getLEDStatus()){
            //calculate current distance with limelight area instead of odometery
            currentDistance = RobotContainer.RC().limelight.getArea(); //need actual fit equation of limelight area vs. distance
        }

    }

    //Low shooting mode = long range = retracted
    //min long range and max short range should not be equal to allow for some historesis to prevent rapid toggling at transition distance
    private void setPositioner(){
        shooterAngleLongRange = !positioner.isDeployed(); //check positioner angle from subsystem
        if ((currentDistance < Shooter.minLongRange) && shooterAngleLongRange) { //below long range, switch to short range
            positioner.deploy();
        } else if ((currentDistance > Shooter.maxShortRange) && !shooterAngleLongRange) { //above short trange, switch to long range
            positioner.retract();
            ;
        }
        shooterAngleLongRange = !positioner.isDeployed(); //check positioner angle from subsystem
    }

    private void calculateVelocity(){       
        if (shooterAngleLongRange) {
            calculatedVel = 4.64*currentDistance + 26.8; //distnce vs. velocity trendline for long range positioner
        } else {
            calculatedVel = 8.5 *currentDistance + 26.5; //distnce vs. velocity trendline for short range positioner
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
            nte_BallVel.setDouble(calculatedVel);
            nte_shooterState.setString(stage.toString());
            nte_distance.setDouble(currentDistance);
            nte_outOfRange.setBoolean(outOfRange);
        }
    }


}
