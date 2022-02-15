package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CAN;
import frc.robot.Constants.ClimbSettings;

public class Climber extends SubsystemBase{
    // NTs
    private NetworkTable table;
    private NetworkTableEntry left_extender_speed, right_extender_speed, left_extender_position, right_extender_position;

    // PIDSlot used
    int slot = 0;

    // raise/lower controllers
    private CANSparkMax left_motor_ext= new CANSparkMax(CAN.CMB_LEFT_Extend, MotorType.kBrushless);
    private CANSparkMax right_motor_ext = new CANSparkMax(CAN.CMB_RIGHT_Extend, MotorType.kBrushless);
    private SparkMaxPIDController left_pidController_ext;
    private SparkMaxPIDController right_pidController_ext;
    private RelativeEncoder left_Encoder_ext;
    private RelativeEncoder right_Encoder_ext;
    private PWM left_PWM_rot;
    private PWM right_PWM_rot;
    private Counter left_Counter_rot;
    private Counter right_Counter_rot;
    private ArmRotation left_Arm;
    private ArmRotation right_Arm;


    // rotation arm controller (outer arms rotate)
    // private CANSparkMax l_rotator = new CANSparkMax(CAN.CMB_L_Rotate, MotorType.kBrushless);
    // private CANSparkMax r_rotator = new CANSparkMax(CAN.CMB_R_Rotate, MotorType.kBrushless);

    public Climber() {
        // TODO - raise/lower are actions ie commands. In the sub-sys we are defining
        // devices
        // and behaviors. Left/Right Inner/Outer arms left-rotator, right-rotator
        // gives a better idea of what

        // TODO Set the control type and PID settings - most likely position control

        // Copy the PID settings down to the SparkMax hardware 
        // Rotation PIDS
        // ClimbSettings.armPID.copyTo(l_rotator.getPIDController(), slot);
        // ClimbSettings.armPID.copyTo(r_rotator.getPIDController(), slot);
        // // Arm extension PIDS
        ClimbSettings.innerPID.copyTo(left_motor_ext.getPIDController(), slot);
        ClimbSettings.innerPID.copyTo(right_motor_ext.getPIDController(), slot);

        // NT stuff
        table = NetworkTableInstance.getDefault().getTable("Climber");
        left_extender_speed = table.getEntry("Left Extender Speed");
        right_extender_speed = table.getEntry("Right Extender Speed");
        left_extender_position = table.getEntry("Left Extender Position");
        right_extender_position = table.getEntry("Right Extender Position");

        left_pidController_ext = left_motor_ext.getPIDController();
        right_pidController_ext = right_motor_ext.getPIDController();
        left_Encoder_ext = left_motor_ext.getEncoder();
        right_Encoder_ext = right_motor_ext.getEncoder();

        left_Encoder_ext.setPosition(0);
        right_Encoder_ext.setPosition(0);     
        

        left_PWM_rot = new PWM(0); //PWM pin 0
        right_PWM_rot = new PWM(1); //PWM pin 1
        left_Counter_rot = new Counter(Counter.Mode.kExternalDirection); //Setting mode of counter to external direction https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/counters.html
        right_Counter_rot = new Counter(Counter.Mode.kExternalDirection); 
        left_Counter_rot.setUpSource(0); //DIO pin 0
        right_Counter_rot.setUpSource(1); //DIO pin 1
        left_Counter_rot.clearDownSource(); // tricking the systems that we only have one channel encoder
        right_Counter_rot.clearDownSource(); // tricking the systems that we only have one channel encoder
        left_Counter_rot.reset(); //resets the count
        right_Counter_rot.reset(); //resets the count
        // .01(1%) is the speed and 2 degrees is the tolerance
        left_Arm = new ArmRotation(left_Counter_rot, left_PWM_rot, .01, 2, table.getSubTable("left_arm_rotation"));
        right_Arm = new ArmRotation(right_Counter_rot, right_PWM_rot, .01, 2, table.getSubTable("left_arm_rotation"));
    }


    public boolean readyToClimb() {
        // returns if robot is in the right position, and all motors are in place
        return false;
    }

    public String rung() {
        return "blank";
    }

    public boolean readyToTraverse() {
        return false;
    }
    // @param inches from extender absolute position
    public void setExtension(double inches) {
        float count =  (float) inches; // convert command inches to encoder counts
        left_pidController_ext.setReference(count, CANSparkMax.ControlType.kPosition);
        right_pidController_ext.setReference(count, CANSparkMax.ControlType.kPosition);
    }
    /**
     * Left and Right arms are controlled in pairs
     * L/R Inner arms move together
     * L/R Outer arms move together
     * 
     * @param inches from retracted position
     */
    // for raising/lower
    // public void setInnerExtension(double inches) {
    //     // negative values lower
    //     // assumed vertical
    // }

    /**
     * Left and Right arms are controlled in pairs
     * L/R Outer arms move together
     * 
     * @param inches from retracted position
     */
    // public void setOuterExtension(double inches) {
    //     // negative values lower
    //     // assumed vertical
    // }

    /**
     * Outer L/R arms rotate together
     * 
     * @param degrees +/- degrees from vertical
     */
    public void setRotation(double degrees) {

        left_Arm.set((int)degrees);
        right_Arm.set((int)degrees);

        // changes the angle of the ? by this many degrees
        // l_rotator.getPIDController().setReference(degrees, ControlType.kPosition);
        // r_rotator.getPIDController().setReference(degrees, ControlType.kPosition);

    }

    public void stop() {
        left_motor_ext.set(0);
        right_motor_ext.set(0);
    }

    public void setSpeed(double left, double right) {
        left_motor_ext.set(left);
        right_motor_ext.set(right);
    }

    public void periodic() {
        // NT updates
        left_extender_speed.setDouble(left_motor_ext.getEncoder().getVelocity());
        right_extender_speed.setDouble(right_motor_ext.getEncoder().getVelocity());
        left_extender_position.setDouble(left_motor_ext.getEncoder().getPosition());
        right_extender_position.setDouble(right_motor_ext.getEncoder().getPosition());
    }

    public RelativeEncoder getLeftEncoder() {
        return left_motor_ext.getEncoder();
    }

    public RelativeEncoder getRightEncoder() {
        return right_motor_ext.getEncoder();
    }
}


class ArmRotation {
    private Counter m_counter;
    private PWM m_motor;
    private double speed;
    private double tolerance;

    private int absPositon = 0;
    private int desPosition = 0;
    private Boolean kForward = true;
    private NetworkTableEntry sdb_desired;
    private NetworkTableEntry sdb_actual;

    public ArmRotation(Counter m_counter, PWM m_motor, double speed, double tolerance, NetworkTable table) {
        this.m_counter = m_counter;
        this.m_motor = m_motor;
        this.speed = speed;
        this.tolerance = tolerance;
        this.sdb_desired = table.getEntry("desired");
        this.sdb_actual = table.getEntry("actual");
    }


    public void periodic() {
        // add the adjustment
        double factor = 1;
        if (!kForward) factor = -1;
        absPositon += factor * m_counter.get();
        m_counter.reset();

        if(Math.abs(desPosition - absPositon ) > tolerance) {
            m_motor.setSpeed(factor * speed);
        } else {
            m_motor.setSpeed(0);
        }
        sdb_actual.setDouble(absPositon);
    }

    public void set(int desired) {
        kForward = desired > absPositon;
        desPosition = desired;
        sdb_desired.setDouble(desPosition);
    }
}