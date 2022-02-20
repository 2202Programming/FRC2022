package frc.robot.subsystems;

import static frc.robot.Constants.CAN;
import static frc.robot.Constants.PCM1;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
    final TalonSRX intake_mtr = new TalonSRX(CAN.INTAKE_TALON);
    final DoubleSolenoid intake_solenoid = new DoubleSolenoid(CAN.PCM1,
                PneumaticsModuleType.REVPH, 
                PCM1.INTAKE_UP_SOLENOID_PCM, 
                PCM1.INTAKE_DOWN_SOLENOID_PCM);
    final DigitalInput intake_lightGate = new DigitalInput(DigitalIO.INTAKE_GATE);

    private CANSparkMax r_belt = new CANSparkMax(CAN.MAG_R_BELT, MotorType.kBrushless);
    private CANSparkMax l_belt = new CANSparkMax(CAN.MAG_L_BELT, MotorType.kBrushless);

    //Constructor
    public Intake_Subsystem(){
        Intake.r_beltPIDF.copyTo(r_belt.getPIDController(), slot);
        Intake.l_beltPIDF.copyTo(l_belt.getPIDController(), slot);
        r_belt.clearFaults();
        r_belt.restoreFactoryDefaults();
        r_belt.setInverted(true);
    
        l_belt.clearFaults();
        l_belt.restoreFactoryDefaults();
        l_belt.setInverted(false);
    }

    //Set the Intake Mode

    //Turn Intake Motor On by sending a double value
    public void on(double intakeMotorStrength, double sideMotorStrength) {
        intake_mtr.set(TalonSRXControlMode.PercentOutput, intakeMotorStrength);
        r_belt.set(sideMotorStrength);
        l_belt.set(sideMotorStrength);
    }

    public void defaultOn(){
        double intakeMotorStrength = 0.47;
        double sideMotorStrength = 0.2;
        intake_mtr.set(TalonSRXControlMode.PercentOutput, intakeMotorStrength);
        r_belt.set(sideMotorStrength);
        l_belt.set(sideMotorStrength);
    }

    //Turn Intake Motor Off by setting a double value
    public void off() {
        intake_mtr.set(TalonSRXControlMode.PercentOutput, 0.0);
        r_belt.set(0);
        l_belt.set(0);
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
}
