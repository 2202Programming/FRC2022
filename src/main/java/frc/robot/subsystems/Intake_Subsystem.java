package frc.robot.subsystems;

import frc.robot.Constants.Intake;
import static frc.robot.Constants.PWM;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake_Subsystem extends SubsystemBase {

    //Instantiations
    final Spark intake_spark = new Spark(PWM.INTAKE);
    final DoubleSolenoid intake_solenoid = new DoubleSolenoid(Intake.INTAKE_PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Intake.INTAKE_UP_SOLENOID_PCM, Intake.INTAKE_DOWN_SOLENOID_PCM);

    //Constructor
    public Intake_Subsystem(){}

    // TODO example API - if too complex or has timing, create commands 
    // and use a more detailed api in command.


    //Deploy arm mechanism using a double solenoids
    public void deploy() {
        intake_solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    //Retract arm mechanism using a double solenoids
    public void retract() {
        intake_solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
}
