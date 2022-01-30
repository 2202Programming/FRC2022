package frc.robot.subsystems;

import frc.robot.Constants.CAN;
import static frc.robot.Constants.PWM;
import static frc.robot.Constants.PCM1;
import static frc.robot.Constants.DigitalIO;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;


public class Intake_Subsystem extends SubsystemBase {
    /**
     * Intake arm consists of a deploy/retractable arm, controlled by Double Solenoid
     * Intake itself consists of a wheel, controlled by Spark PWM value
     */

    //Localized Constants - what valve value does what action
    static final Value DEPLOY  = Value.kReverse;
    static final Value RETRACT = Value.kForward;
    private boolean intakeIsOn = false;
    //Instantiations
    final Spark intake_spark = new Spark(PWM.INTAKE);
    final DoubleSolenoid intake_solenoid = new DoubleSolenoid(CAN.PCM1,
                PneumaticsModuleType.CTREPCM, 
                PCM1.INTAKE_UP_SOLENOID_PCM, 
                PCM1.INTAKE_DOWN_SOLENOID_PCM);
    final DigitalInput intake_lightGate = new DigitalInput(DigitalIO.INTAKE_GATE);

    //Constructor
    public Intake_Subsystem(){}

    //Set the Intake Mode

    //Turn Intake Motor On by sending a double value
    public void on(double motorStrength) {
        intake_spark.set(motorStrength);
    }

    //Turn Intake Motor Off by setting a double value
    public void off() {
        intake_spark.set(0);
    }

    //Turn Intake Motor in reverse by sending a double value
    public void expell(double motorStrength){
        intake_spark.set(-motorStrength);
    }

    //Deploy arm mechanism using a Double Solenoids
    public void deploy() {
        intake_solenoid.set(DEPLOY);
    }

    //Retract arm mechanism using a Double Solenoids
    public void retract() {
        intake_solenoid.set(RETRACT);
    }
    
    //Indicates if Cargo is inside the intake
    public boolean isCargoDetected() {
        return intake_lightGate.get();
    }

    //Returns the state of the Intake Arm
    public boolean isDeployed() {
      return ( intake_solenoid.get() == DEPLOY); 
    }

    //TODO - how is this different than off()  above? ALSO, prefer shorter off/on 
    // to avoid intake.intakeOff()    intake.off()   reads better
    //TODO  is intakeIsOn variable needed, could you create a functions
    // that looks at the motor state?
    public void intakeOff() {
        intakeIsOn = false;
        intake_spark.set(0);
    }
}
