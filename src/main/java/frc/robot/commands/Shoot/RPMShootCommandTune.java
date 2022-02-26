package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.FlyWheelRPM;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;

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

    FlyWheelRPM prevRPM;

    private boolean finished = false;

    private double upperP;
    private double upperI;
    private double upperD;
    private double lowerP;
    private double lowerI;
    private double lowerD;

    private double requestedVelocity = 10;
    private double previousVelocity = 10;

    ShooterSettings  cmdSS;         // instance the shooter sees

    final ShooterSettings defaultShooterSettings = new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.01);

    private BasicShootCommand currentShooterCommand;

    public RPMShootCommandTune(double requestedVelocity){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        ShooterSettings target = new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.01);
        this.cmdSS = target;
    }
    
    public RPMShootCommandTune(){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        cmdSS = defaultShooterSettings;
    }

    @Override
    public void initialize(){
        System.out.println("***Shooter Tune init***");
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");

        currentShooterCommand = new BasicShootCommand(new ShooterSettings(10, 0.0, USE_CURRENT_ANGLE, 0.01));
        CommandScheduler.getInstance().schedule(currentShooterCommand);

        //shooter.off();
    }

    @Override
    public void execute(){
        checkDashboard();
        getPID();
        checkPID();
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

        SmartDashboard.putNumber("Velocity Requested", requestedVelocity);
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
        requestedVelocity = SmartDashboard.getNumber("Velocity Requested", 10);
        cmdSS = new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.01);
        if(requestedVelocity != previousVelocity){
            currentShooterCommand.setFinished();
            currentShooterCommand = new BasicShootCommand(new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.01)); 
            CommandScheduler.getInstance().schedule(currentShooterCommand);
        }
        previousVelocity = requestedVelocity;
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("***Shooter Tune end***");
        currentShooterCommand.setFinished();
    }

    public void setFinished(){
        finished = true;
    }
    
    @Override
    public boolean isFinished(){
        return finished;
    }
}
