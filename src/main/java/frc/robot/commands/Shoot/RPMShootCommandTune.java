package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.FlyWheelRPM;
import frc.robot.subsystems.shooter.Shooter_Subsystem;

public class RPMShootCommandTune extends CommandBase{ 
    public static final double USE_CURRENT_ANGLE = 0.0;

    final Magazine_Subsystem magazine;
    final Intake_Subsystem intake;
    final Shooter_Subsystem shooter;
    final double TESTANGLE = 0.0;
    final double TESTTOL = 0.02;
    int ballCount = 999;
    int backupCounter = 0;

    NetworkTable table;
    NetworkTableEntry shooterState;

    FlyWheelRPM cmdRPM;
    FlyWheelRPM prevRPM;

    private boolean finished = false;

    private double upperP;
    private double upperI;
    private double upperD;
    private double lowerP;
    private double lowerI;
    private double lowerD;


    final FlyWheelRPM defaultShooterRPMs = new FlyWheelRPM(1000,1000);

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

    public RPMShootCommandTune(FlyWheelRPM target){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        this.cmdRPM = target;
    }
    
    public RPMShootCommandTune(){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        cmdRPM = defaultShooterRPMs;
    }

    @Override
    public void initialize(){
        System.out.println("***Shooter Tune init***");
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");
        shooterState = table.getEntry("ShooterState");

        stage = Stage.DoNothing;
        shooter.off();

    }

    @Override
    public void execute(){
        System.out.println("***Shooter Tune execute***");
        shooterState.setString(stage.toString());
        checkDashboard();
        getPID();
        checkPID();
        switch(stage){
            case DoNothing:
                magazine.driveWheelOff();
                shooter.on(cmdRPM);
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

    private void getPID(){
        upperP = shooter.getUpperP();
        upperI = shooter.getUpperI();
        upperD = shooter.getUpperD();
        lowerP = shooter.getlowerP();
        lowerI = shooter.getlowerI();
        lowerD = shooter.getlowerD();
        SmartDashboard.putNumber("Current Upper P", upperP);
        SmartDashboard.putNumber("Current Upper I", upperI);
        SmartDashboard.putNumber("Current Upper D", upperD);
        SmartDashboard.putNumber("Current Lower P", lowerP);
        SmartDashboard.putNumber("Current Lower I", lowerI);
        SmartDashboard.putNumber("Current Lower D", lowerD);
    }

    private void checkPID(){
        if (upperP != SmartDashboard.getNumber("Requested Upper P", upperP)){
            shooter.setPIDUpper(SmartDashboard.getNumber("Requested Upper P", upperP), upperI, upperD);
        }
        if (upperI != SmartDashboard.getNumber("Requested Upper I", upperI)){
            shooter.setPIDUpper(upperP, SmartDashboard.getNumber("Requested Upper I", upperI), upperD);
        }
        if (upperD != SmartDashboard.getNumber("Requested Upper D", upperD)){
            shooter.setPIDUpper(upperP, upperI, SmartDashboard.getNumber("Requested Upper D", upperD));
        }
        if (lowerP != SmartDashboard.getNumber("Requested lower P", lowerP)){
            shooter.setPIDLower(SmartDashboard.getNumber("Requested lower P", lowerP), lowerI, lowerD);
        }
        if (lowerI != SmartDashboard.getNumber("Requested lower I", lowerI)){
            shooter.setPIDLower(lowerP, SmartDashboard.getNumber("Requested lower I", lowerI), lowerD);
        }
        if (lowerD != SmartDashboard.getNumber("Requested lower D", lowerD)){
            shooter.setPIDLower(lowerP, lowerI, SmartDashboard.getNumber("Requested lower D", lowerD));
        }
    }

    private void checkDashboard(){
        FlyWheelRPM actualRPMs = new FlyWheelRPM();
        shooter.getFlyWheelRPM(actualRPMs);
        SmartDashboard.putNumber("Current Upper RPM", actualRPMs.upper);
        SmartDashboard.putNumber("Current Lower RPM", actualRPMs.lower);
        SmartDashboard.putString("Shooting Stage", stage.toString());
        cmdRPM = new FlyWheelRPM(SmartDashboard.getNumber("Upper RPM Requested", 1000), SmartDashboard.getNumber("Lower RPM Requested", 1000));
    }

    @Override
    public void end(boolean interrupted){
        stage = Stage.DoNothing;
        magazine.driveWheelOff();
        shooter.off();
        System.out.println("***Shooter Tune end***");
    }

    public void setFinished(){
        finished = true;
    }
    
    @Override
    public boolean isFinished(){
        return finished;
    }
}
