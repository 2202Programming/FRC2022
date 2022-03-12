package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.FlyWheelRPM;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;
import frc.robot.util.PoseMath;

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

    private double FF;

    private double r_upperP = 0.2;
    private double r_upperI = 0.000005;
    private double r_upperD = 0.0;

    private double r_FF = 0;

    private double requestedVelocity = 25;
    private double previousVelocity = 25;

    ShooterSettings  cmdSS;         // instance the shooter sees

    final ShooterSettings defaultShooterSettings = new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.20);

    private VelShootCommand currentShooterCommand;
    private Pose2d centerField = Constants.Autonomous.hubPose;
    private double distanceToTarget = 0;
    
    public RPMShootCommandTune(double requestedVelocity){
        this.intake = RobotContainer.RC().intake;
        this.shooter = RobotContainer.RC().shooter;
        this.magazine = RobotContainer.RC().magazine;
        ShooterSettings target = new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.20);
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
        table = NetworkTableInstance.getDefault().getTable("ShootCommand");

        currentShooterCommand = new VelShootCommand(new ShooterSettings(10, 0.0) , 20);
        CommandScheduler.getInstance().schedule(currentShooterCommand);
        RobotContainer.RC().drivetrain.setPose(Autonomous.startPose1);

        SmartDashboard.putNumber("Requested Flywheel P", r_upperP);
        SmartDashboard.putNumber("Requested Flywheel I", r_upperI);
        SmartDashboard.putNumber("Requested Flywheel D", r_upperD);
        SmartDashboard.putNumber("requested FF", r_FF);
        CommandScheduler.getInstance().schedule(new IntakeCommand((()-> 0.47), ()-> 0.30,  IntakeMode.LoadCargo));
    }

    @Override
    public void execute(){
        checkDashboard();
        getPID();
        checkPID();
        distanceToTarget = PoseMath.poseDistance(RobotContainer.RC().drivetrain.getPose(), Autonomous.hubPose);
    }

    private void getPID(){
        upperP = shooter.getUpperP();
        upperI = shooter.getUpperI();
        upperD = shooter.getUpperD();
    
        SmartDashboard.putNumber("Current Flywheel P", upperP);
        SmartDashboard.putNumber("Current Flywheel I", upperI);
        SmartDashboard.putNumber("Current Flywheel D", upperD);
        SmartDashboard.putNumber("Current FF", FF);


        SmartDashboard.putNumber("Velocity Requested", requestedVelocity);
        FlyWheelRPM flyWheelRPM = new FlyWheelRPM();
        shooter.getFlyWheelRPM(flyWheelRPM);
        SmartDashboard.putNumber("Upper RPM", flyWheelRPM.upper);
        SmartDashboard.putNumber("Lower RPM", flyWheelRPM.lower);
        SmartDashboard.putNumber("DistanceToTarget", distanceToTarget);
    }

    private void checkPID(){
        r_upperP = SmartDashboard.getNumber("Requested Flywheel P", upperP);
        r_upperI = SmartDashboard.getNumber("Requested Flywheel I", upperI);
        r_upperD = SmartDashboard.getNumber("Requested Flywheel D", upperD);
        r_FF = SmartDashboard.getNumber("Requested FF", FF);
        if (upperP != r_upperP){
            shooter.setPIDUpper(r_upperP, upperI, upperD);
            shooter.setPIDLower(r_upperP, upperI, upperD);
        }
        if (upperI != r_upperI){
            shooter.setPIDUpper(upperP, r_upperI, upperD);
            shooter.setPIDLower(upperP, r_upperI, upperD);
        }
        if (upperD != r_upperD){
            shooter.setPIDUpper(upperP, upperI, r_upperD);
            shooter.setPIDLower(upperP, upperI, r_upperD);
        }
    }

    private void checkDashboard(){
        requestedVelocity = SmartDashboard.getNumber("Velocity Requested", 10);  
        if(requestedVelocity != previousVelocity){
            currentShooterCommand.setFinished();
            currentShooterCommand = new VelShootCommand(new ShooterSettings(requestedVelocity, 0.0, USE_CURRENT_ANGLE, 0.1), 20 ); // backup count frames 
            System.out.println("**Requested velocity: " + requestedVelocity);
            CommandScheduler.getInstance().schedule(currentShooterCommand);
        }
        previousVelocity = requestedVelocity;
    }

    @Override
    public void end(boolean interrupted){
        currentShooterCommand.setFinished();
        intake.off();
    }

    public void setFinished(){
        finished = true;
    }
    
    @Override
    public boolean isFinished(){
        return finished;
    }


    
}
