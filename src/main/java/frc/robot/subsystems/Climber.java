package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CAN;
import frc.robot.Constants.ClimbSettings;


public class Climber extends SubsystemBase {
    // NTs
    private NetworkTable table;
    private NetworkTableEntry left_extender_speed, right_extender_speed, left_extender_position, right_extender_position, left_reverse_open, right_reverse_open, left_rotation_limit, right_rotation_limit, left_forward_open, right_forward_open;

    // PIDSlot used
    int slot = 0;

    // raise/lower controllers
    private CANSparkMax left_motor_ext= new CANSparkMax(CAN.CMB_LEFT_Extend, MotorType.kBrushless);
    private CANSparkMax right_motor_ext = new CANSparkMax(CAN.CMB_RIGHT_Extend, MotorType.kBrushless);
    private SparkMaxPIDController left_pidController_ext;
    private SparkMaxPIDController right_pidController_ext;


    // Limit switches extension
    // TODO: Is type kNormallyClosed or kNormallyOpen? Also check for rotation limit swithces
    // TODO: Implement what happens if limit switch is true
    /**
     * Note: there are 3 limit switches --
     *  1 limit switch to check if extension is at top
     *  1 limit switch to check if extension is at bottom
     *  1 limit switch to check if rotation is vertical
     */

    private SparkMaxLimitSwitch leftForwardLimitSwitch = left_motor_ext.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    private SparkMaxLimitSwitch rightForwardLimitSwitch = right_motor_ext.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

    private SparkMaxLimitSwitch leftReverseLimitSwitch = left_motor_ext.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    private SparkMaxLimitSwitch rightReverseLimitSwitch = right_motor_ext.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    
    
    private ArmRotation left_Arm;
    private ArmRotation right_Arm;
    private CANSparkMax left_motor_rot = new CANSparkMax(CAN.CMB_LEFT_Rotate, MotorType.kBrushed);
    private CANSparkMax right_motor_rot = new CANSparkMax(CAN.CMB_RIGHT_Rotate, MotorType.kBrushed);
    private SparkMaxPIDController left_pidController_rot;
    private SparkMaxPIDController right_pidController_rot;

    // Limit switches rotation
    private SparkMaxLimitSwitch leftRotationLimitSwitch = left_motor_rot.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    private SparkMaxLimitSwitch rightRotationLimitSwitch = right_motor_rot.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    boolean leftRotationIsPressed;
    boolean rightRotationIsPressed;
    boolean leftForwardLimitSwitchIsPressed;
    boolean rightForwardLimitSwitchIsPressed;
    boolean leftReverseIsPressed;
    boolean rightReverseIsPressed;
    



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
        
        
        
        
        ClimbSettings.extendPID.copyTo(left_motor_ext.getPIDController(), slot);
        ClimbSettings.extendPID.copyTo(right_motor_ext.getPIDController(), slot);
        right_motor_ext.setInverted(true);

        ClimbSettings.rotatePID.copyTo(left_motor_rot.getPIDController(), slot);
        ClimbSettings.rotatePID.copyTo(right_motor_rot.getPIDController(), slot);
        right_motor_rot.setInverted(true);

        // NT stuff
        
        table = NetworkTableInstance.getDefault().getTable("Climber");
        left_extender_speed = table.getEntry("Left Extender Speed");
        right_extender_speed = table.getEntry("Right Extender Speed");
        
        left_extender_position = table.getEntry("Left Extender Position");
        right_extender_position = table.getEntry("Right Extender Position");
        
        left_reverse_open = table.getEntry("Left Reverse Limit Enabled");
        right_reverse_open = table.getEntry("Right Reverse Limit Enabled");
        
        left_forward_open = table.getEntry("Left Forward Limit Enabled");
        right_forward_open = table.getEntry("Right Forward Limit Enabled");
        
        left_rotation_limit = table.getEntry("Left Rotation Limit Enabled");
        right_rotation_limit = table.getEntry("Right Rotation Limit Enabled");

        left_pidController_ext = left_motor_ext.getPIDController();
        right_pidController_ext = right_motor_ext.getPIDController();
        
        left_motor_ext.getEncoder().setPosition(0);
        right_motor_ext.getEncoder().setPosition(0);

        left_pidController_rot = left_motor_ext.getPIDController();
        right_pidController_rot = right_motor_ext.getPIDController();
 
        left_motor_rot.getEncoder().setPosition(0);
        right_motor_rot.getEncoder().setPosition(0);

        left_Arm = new ArmRotation(table.getSubTable("left_arm_rotation"), left_pidController_rot);
        right_Arm = new ArmRotation(table.getSubTable("right_arm_rotation"), right_pidController_rot);
        
        leftForwardLimitSwitch.enableLimitSwitch(leftForwardLimitSwitch.isPressed());
        rightForwardLimitSwitch.enableLimitSwitch(rightForwardLimitSwitch.isPressed());

        leftReverseLimitSwitch.enableLimitSwitch(leftReverseLimitSwitch.isPressed());
        rightReverseLimitSwitch.enableLimitSwitch(rightReverseLimitSwitch.isPressed());

        leftRotationLimitSwitch.enableLimitSwitch(leftRotationLimitSwitch.isPressed());
        rightRotationLimitSwitch.enableLimitSwitch(rightRotationLimitSwitch.isPressed());

        leftRotationIsPressed = leftRotationLimitSwitch.isPressed();
        rightRotationIsPressed = rightRotationLimitSwitch.isPressed();

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
        double count = ((25.4*36*42)/(5*24))*inches; //25.4 is mm per inch, 36 is revs per 1 pev with gearbox, 42 is counts per rev and 5 is mm per 1 tooth
        //   42 Counts per rev, but SmartMotion was showing rotations not counts
        //   Connected to a 36:1 gearbox that could change
        //   Gearbox connected to a 24 tooth pulley that is connected to a linear belt
        //   Need to ask mechs to do analysis to convert 24 tooth pulley to linear distance

        
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
        double counts = (42*174.9*degrees)/(360*20); //42 and 20 are the tooth pullies, 174.9 is the counts per rot and 360 is the degrees
        // 174.9 counter/rotation on motor
        // Motor is connected to a 20 tooth pulley that drives
        //    a 42 tooth pulley
        // changes the angle of the ? by this many degrees
        // l_rotator.getPIDController().setReference(degrees, ControlType.kPosition);
        // r_rotator.getPIDController().setReference(degrees, ControlType.kPosition);
        left_Arm.set((float)counts);
        right_Arm.set((float)counts);
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
        // Sets limit switch enabled
        leftRotationIsPressed = leftRotationLimitSwitch.isPressed();
        rightRotationIsPressed = rightRotationLimitSwitch.isPressed();
        
        leftForwardLimitSwitchIsPressed = rightRotationLimitSwitch.isPressed();
        rightForwardLimitSwitchIsPressed = rightRotationLimitSwitch.isPressed();
        
        leftReverseIsPressed = leftReverseLimitSwitch.isPressed();
        rightReverseIsPressed = rightReverseLimitSwitch.isPressed();
        
        leftRotationLimitSwitch.enableLimitSwitch(leftRotationIsPressed);
        rightRotationLimitSwitch.enableLimitSwitch(rightRotationIsPressed);
        
        leftForwardLimitSwitch.enableLimitSwitch(leftForwardLimitSwitchIsPressed);
        rightForwardLimitSwitch.enableLimitSwitch(rightForwardLimitSwitchIsPressed);
        
        leftReverseLimitSwitch.enableLimitSwitch(leftReverseIsPressed);
        rightReverseLimitSwitch.enableLimitSwitch(rightReverseIsPressed);


        // NT updates
        left_extender_speed.setDouble(left_motor_ext.getEncoder().getVelocity());
        right_extender_speed.setDouble(right_motor_ext.getEncoder().getVelocity());
        
        left_extender_position.setDouble(left_motor_ext.getEncoder().getPosition());
        right_extender_position.setDouble(right_motor_ext.getEncoder().getPosition());
        
        left_reverse_open.setBoolean(leftReverseLimitSwitch.isPressed());
        right_reverse_open.setBoolean(rightReverseLimitSwitch.isPressed());
        
        left_forward_open.setBoolean(leftForwardLimitSwitch.isPressed());
        right_forward_open.setBoolean(rightForwardLimitSwitch.isPressed());
        
        left_rotation_limit.setBoolean(leftRotationIsPressed);
        right_rotation_limit.setBoolean(rightRotationIsPressed);

        left_Arm.periodic();
        right_Arm.periodic();
    }

    public RelativeEncoder getLeftEncoder() {
        return left_motor_ext.getEncoder();
    }

    public RelativeEncoder getRightEncoder() {
        return right_motor_ext.getEncoder();
    }
}

class ArmRotation {
    private NetworkTableEntry sdb_desired;
    private SparkMaxPIDController pidController_rot;

    public ArmRotation(NetworkTable table, SparkMaxPIDController pidController_rot) {
        this.pidController_rot = pidController_rot;
        this.sdb_desired = table.getEntry("desired");
    }
    public void periodic() {

    }

    public void set(double counts) {
        pidController_rot.setReference(counts, CANSparkMax.ControlType.kPosition);
        sdb_desired.setDouble(counts);
    }
}