package frc.robot.subsystems;

import frc.robot.Constants.Intake;
import static frc.robot.Constants.PWM;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake_Subsystem extends SubsystemBase {

    // TODO example API - if too complex or has timing, create commands 

    //Instantiations
    final Spark intake_spark = new Spark(PWM.INTAKE);
    final DoubleSolenoid intake_solenoid = new DoubleSolenoid(Intake.INTAKE_PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Intake.INTAKE_UP_SOLENOID_PCM, Intake.INTAKE_DOWN_SOLENOID_PCM);

    //Constructor
    public Intake_Subsystem(){}

    //Turn Intake Motor On by sending a double value
    public void intakeOn(double motorStrength) {
        intake_spark.set(motorStrength);
    }

    //Turn Intake Motor Off by sending a doulbe value
    public void intakeOff() {
        intake_spark.set(0);
    }

    //Deploy arm mechanism using a Double Solenoids
    public void deploy() {
        intake_solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    //Retract arm mechanism using a Double Solenoids
    public void retract() {
        intake_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
}
