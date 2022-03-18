package frc.robot.subsystems;

import static frc.robot.Constants.CAN;
import static frc.robot.Constants.PCM1;

import static frc.robot.Constants.DigitalIO;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Intake;

public class Intake_Subsystem extends SubsystemBase {
    /**
     * Intake arm consists of a deploy/retractable arm, controlled by Double Solenoid
     * Intake itself consists of a wheel, controlled by Spark PWM value
     */

    //Localized Constants - what valve value does what action
    static final Value DEPLOY  = Value.kReverse;
    static final Value RETRACT = Value.kForward;

    //slot to use on controllers
    int slot = 0;

    //Instantiations
    final CANSparkMax intake_mtr = new CANSparkMax(CAN.INTAKE_MTR, CANSparkMax.MotorType.kBrushless);
    final DoubleSolenoid intake_solenoid = new DoubleSolenoid(CAN.PCM1,
                PneumaticsModuleType.CTREPCM,
                PCM1.INTAKE_UP_SOLENOID_PCM,
                PCM1.INTAKE_DOWN_SOLENOID_PCM);
    final DigitalInput intake_lightGate = new DigitalInput(DigitalIO.INTAKE_GATE);

    private CANSparkMax r_side_mtr = new CANSparkMax(CAN.MAG_R_SIDE_MTR, MotorType.kBrushless);
    private CANSparkMax l_side_mtr = new CANSparkMax(CAN.MAG_L_SIDE_MTR, MotorType.kBrushless);

    //Constructor
    public Intake_Subsystem(){
        Intake.r_side_mtrPIDF.copyTo(r_side_mtr.getPIDController(), slot);
        Intake.l_side_mtrPIDF.copyTo(l_side_mtr.getPIDController(), slot);
        r_side_mtr.clearFaults();
        r_side_mtr.restoreFactoryDefaults();
        r_side_mtr.setInverted(true);
    
        l_side_mtr.clearFaults();
        l_side_mtr.restoreFactoryDefaults();
        l_side_mtr.setInverted(false);

        off(); // Stops intake from running after auto ends
    }

    //Set the Intake Mode

    //Turn Intake Motor On by sending a double value
    public void on(double intakeMotorStrength, double sideMotorStrength) {
        intake_mtr.set(intakeMotorStrength);
        r_side_mtr.set(sideMotorStrength);
        l_side_mtr.set(sideMotorStrength);
    }

    public void defaultOn(){
        double intakeMotorStrength = 0.47;
        double sideMotorStrength = 0.2;
        intake_mtr.set(intakeMotorStrength);
        r_side_mtr.set(sideMotorStrength);
        l_side_mtr.set(sideMotorStrength);
    }

    //Turn Intake Motor Off by setting a double value
    public void off() {
        intake_mtr.set(0.0);
        r_side_mtr.set(0);
        l_side_mtr.set(0);
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
}
