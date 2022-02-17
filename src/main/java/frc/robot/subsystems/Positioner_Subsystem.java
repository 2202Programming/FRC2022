package frc.robot.subsystems;

import static frc.robot.Constants.CAN;
import static frc.robot.Constants.PCM1;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Positioner_Subsystem extends SubsystemBase {
    /**
     * Intake arm consists of a deploy/retractable arm, controlled by Double Solenoid
     * Intake itself consists of a wheel, controlled by Spark PWM value
     */

    //Localized Constants - what valve value does what action
    static final Value UPWARD = Value.kReverse;
    static final Value DOWNWARD = Value.kForward;
    //Instantiations
    final DoubleSolenoid positioner_solenoid = new DoubleSolenoid(CAN.PCM1,
                PneumaticsModuleType.CTREPCM, 
                PCM1.POSITIONER_UP_SOLENOID_PCM, 
                PCM1.POSITIONER_DOWN_SOLENOID_PCM);

    //Constructor
    public Positioner_Subsystem(){
        
    }

    //Turn Intake Motor On by sending a double value

    //Turn Intake Motor Off by setting a double value

    //Deploy arm mechanism using a Double Solenoids
    public void upward() {
        positioner_solenoid.set(UPWARD);
    }

    //Retract arm mechanism using a Double Solenoids
    public void downward() {
        positioner_solenoid.set(DOWNWARD);
    }

    //Returns the state of the Intake Arm
    public boolean isUpPosition() {
      return ( positioner_solenoid.get() == UPWARD); 
    }

    //TODO - how is this different than off()  above? ALSO, prefer shorter off/on 
    // to avoid intake.intakeOff()    intake.off()   reads better
    //TODO  is intakeIsOn variable needed, could you create a functions
    // that looks at the motor state?
    //Among Us
}
