package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.trainingShooter.Test_FlyWheel.FlyWheelConfig;
import frc.robot.util.PIDFController;

public class Test_Shooter_Subsystem extends SubsystemBase{
    

    // Cargo info definitions from Constants
    public static final double PCNominalRadius = 10 / 2.0 / 12.0; // feet - power cell
    public static final double PCEffectiveRadius = 8 / 2.0 / 12.0; // feet - compressed radius
    public static final double FlyWheelGearRatio = 1;
    //Shooter info from constants
    public static final double DefaultRPMTolerance = .05;  // percent of RPM
    public static final ShooterSettings DefaultSettings = new ShooterSettings(10.0, 0.0);  //ft/s, rot/s


    //Network Tables
    private NetworkTable table;
    private NetworkTableEntry nt_upperRPM;
    private NetworkTableEntry nt_lowerRPM;
    private NetworkTableEntry nt_upperRPMErr;
    private NetworkTableEntry nt_lowerRPMErr;
    
    // Flywheels 
    final Test_FlyWheel  upper_shooter; 
    final Test_FlyWheel  lower_shooter; 


    //This is what the Command Structure interacts with to set the Flywheel Speed
    public static class ShooterSettings{
        //globals
        public double velocity; //Cargo's speed ft/sec
        public double rotationsPerSecond; // Cargo Rotation rotation/sec
        public double angle; //angle in which the Cargo leaves
        public double velocityTolerance; // percent tolerance
        
        //Empty Constructor
        public ShooterSettings(){
            velocity = 0.0;
            rotationsPerSecond = 0.0;
            angle = 0.0;
            velocityTolerance = 0.0;
        }

        //Constructor Passing in all 4 values
        public ShooterSettings(double velocity, double rotationsPerSecond, double angle, double velocityTolerance){
            this.velocity = velocity;
            this.rotationsPerSecond = rotationsPerSecond;
            this.angle = angle;
            this.velocityTolerance = velocityTolerance;
        }

        //Constructor with only velocity and rps
        public ShooterSettings(double velocity, double rotationsPerSecond){
            this.velocity = velocity;
            this.rotationsPerSecond = rotationsPerSecond;
            angle = 0.0;
            velocityTolerance = 0.0;
        }

        //Constructor passing in ShooterSettings
        public ShooterSettings(ShooterSettings shooterSetting){
            this.velocity = shooterSetting.velocity;
            this.rotationsPerSecond = shooterSetting.rotationsPerSecond;
            this.angle = shooterSetting.angle;
            this.velocityTolerance = shooterSetting.velocityTolerance;
        }

        //Maybe a method to make sure 2 Settings are equal
        public boolean isEqual(ShooterSettings shooterSetting){
            if(this.velocity == shooterSetting.velocity && this.rotationsPerSecond == shooterSetting.rotationsPerSecond && this.angle == shooterSetting.angle && this.velocityTolerance == shooterSetting.velocityTolerance ){
                return true;   
            }
            return false;
            
        }
    }

    /*
    * Stuff for our Shooter Subsystem
    */
    // All RPM are in Flywheel-RPM, not motor.
    FlyWheelRPM actualRPM = new FlyWheelRPM();
    FlyWheelRPM targetRPM = new FlyWheelRPM();
    FlyWheelRPM error = new FlyWheelRPM();

    //Transfrom from [ w, V] [W_lower, W_upper]
    final Matrix<N2,N2> VelocityToRPM = new Matrix<>(Nat.N2(), Nat.N2() );
    Vector<N2> velocity = new Vector<N2>(Nat.N2());
    
    //state variables
    private boolean m_readyToShoot = false;  
    ShooterSettings shooter_setpoint;     // reference to current shooter setpoint, angle, flywheel speeds
    


    //Constructor
    public Test_Shooter_Subsystem(){
        //Define new shooter wheels of type FlyWheel


        //Network table stuff
        table = NetworkTableInstance.getDefault().getTable("Test Shooter");

        //Stuff for matrix and Velocity
        VelocityToRPM.set(0,0,0);
        VelocityToRPM.set(0,0,0);
        VelocityToRPM.set(0,0,0);
        VelocityToRPM.set(0,0,0);
        //should we scale ours settings

        //Should we have a default Shooter Settings?
    }

    @Override
    public void periodic(){
        //Periodic executes everytime through the loop
        //What should be do?
        //Network Tables?

        //How can we know we are ready to shoot?

    }

    

    public void log(){
        //what should we log for Debugging?
        //Look at our network tables
    }
}
