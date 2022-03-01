package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ClimbSettings;


public class Climber extends SubsystemBase {
    // NTs
    private NetworkTable table;
    // PIDSlot used
    int slot = 0;

 
    // Limit switches extension
    // TODO: Is type kNormallyClosed or kNormallyOpen? Also check for rotation limit swithces
    // TODO: Implement what happens if limit switch is true
    /**
     * Note: there are 3 limit switches --
     *  1 limit switch to check if extension is at top
     *  1 limit switch to check if extension is at bottom
     *  1 limit switch to check if rotation is vertical
     */

    private CANSparkMax left_motor_rot = new CANSparkMax(CAN.CMB_LEFT_Rotate, MotorType.kBrushed);
    private CANSparkMax right_motor_rot = new CANSparkMax(CAN.CMB_RIGHT_Rotate, MotorType.kBrushed);
    private CANSparkMax left_motor_ext = new CANSparkMax(CAN.CMB_LEFT_Extend, MotorType.kBrushless);
    private CANSparkMax right_motor_ext = new CANSparkMax(CAN.CMB_RIGHT_Extend, MotorType.kBrushless);
    private ArmExtension left_Arm_ext;
    private ArmExtension right_Arm_ext;
    private ArmRotation left_Arm_rot;
    private ArmRotation right_Arm_rot;


    // rotation arm controller (outer arms rotate)
    // private CANSparkMax l_rotator = new CANSparkMax(CAN.CMB_L_Rotate, MotorType.kBrushless);
    // private CANSparkMax r_rotator = new CANSparkMax(CAN.CMB_R_Rotate, MotorType.kBrushless);

    public Climber() {
        table = NetworkTableInstance.getDefault().getTable("Climber");

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
        right_motor_ext.clearFaults();
        right_motor_ext.restoreFactoryDefaults();
        left_motor_ext.clearFaults();
        left_motor_ext.restoreFactoryDefaults();
        
        
        
        ClimbSettings.extendPID.copyTo(left_motor_ext.getPIDController(), slot);
        ClimbSettings.extendPID.copyTo(right_motor_ext.getPIDController(), slot);
        right_motor_ext.setInverted(false);
        left_motor_ext.setInverted(true);
       

        ClimbSettings.rotatePID.copyTo(left_motor_rot.getPIDController(), slot);
        ClimbSettings.rotatePID.copyTo(right_motor_rot.getPIDController(), slot);
        right_motor_rot.setInverted(true);

        // NT stuff
        
        left_extender_speed = table.getEntry("Left Extender Speed");
        right_extender_speed = table.getEntry("Right Extender Speed");
        
        left_extender_position = table.getEntry("Left Extender Position");
        right_extender_position = table.getEntry("Right Extender Position");

        right_extensions_lower_limit = table.getEntry("Right Extensions Lower Limit");
        left_extensions_lower_limit = table.getEntry("Left Extensions Lower Limit");
        right_extensions_upper_limit = table.getEntry("Right Extensions Upper Limit");
        left_extensions_upper_limit = table.getEntry("Left Extensions Upper Limit");
        
        left_reverse_open = table.getEntry("Left Reverse Limit Enabled");
        right_reverse_open = table.getEntry("Right Reverse Limit Enabled");
        
        left_forward_open = table.getEntry("Left Forward Limit Enabled");
        right_forward_open = table.getEntry("Right Forward Limit Enabled");
        
        left_rotation_limit = table.getEntry("Left Rotation Limit Enabled");
        right_rotation_limit = table.getEntry("Right Rotation Limit Enabled");

        arm_extensions_desired_position = table.getEntry("Arm Extensions Desired Position");

        left_pidController_ext = left_motor_ext.getPIDController();
        right_pidController_ext = right_motor_ext.getPIDController();
        
        // left_motor_ext.getEncoder().setPosition(0);
        // right_motor_ext.getEncoder().setPosition(0);
// //  
//         left_motor_rot.getEncoder().setPosition(0);
//         right_motor_rot.getEncoder().setPosition(0);

        // left_Arm = new ArmRotation(table.getSubTable("left_arm_rotation"), left_pidController_rot);
        // right_Arm = new ArmRotation(table.getSubTable("right_arm_rotation"), right_pidController_rot);
        
        // leftForwardLimitSwitch.enableLimitSwitch(leftForwardLimitSwitch.isPressed());
        // rightForwardLimitSwitch.enableLimitSwitch(rightForwardLimitSwitch.isPressed());

        // leftReverseLimitSwitch.enableLimitSwitch(leftReverseLimitSwitch.isPressed());
        // rightReverseLimitSwitch.enableLimitSwitch(rightReverseLimitSwitch.isPressed());

        // leftRotationLimitSwitch.enableLimitSwitch(leftRotationLimitSwitch.isPressed());
        // rightRotationLimitSwitch.enableLimitSwitch(rightRotationLimitSwitch.isPressed());

        // leftRotationIsPressed = leftRotationLimitSwitch.isPressed();
        // rightRotationIsPressed = rightRotationLimitSwitch.isPressed();

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

        arm_extensions_desired_position.setDouble(count);
        
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

        left_extensions_upper_limit.setBoolean(left_motor_ext.getForwardLimitSwitch(Type.kNormallyClosed).isPressed());
        right_extensions_upper_limit.setBoolean(right_motor_ext.getForwardLimitSwitch(Type.kNormallyClosed).isPressed());

        left_extensions_lower_limit.setBoolean(left_motor_ext.getReverseLimitSwitch(Type.kNormallyClosed).isPressed());
        right_extensions_lower_limit.setBoolean(right_motor_ext.getReverseLimitSwitch(Type.kNormallyClosed).isPressed());

        var newamps = (int)extension_amps.getDouble(1);
        if(currentAmps != newamps){ 
            currentAmps = newamps;
            left_motor_ext.setSmartCurrentLimit(newamps);
            right_motor_ext.setSmartCurrentLimit(newamps);
        }
        // left_Arm.periodic();
        // right_Arm.periodic();
    }

    public RelativeEncoder getLeftExtEncoder() {
        return left_motor_ext.getEncoder();
    }

    public RelativeEncoder getRightExtEncoder() {
        return right_motor_ext.getEncoder();
    }

    public RelativeEncoder getLeftRotEncoder() {
        return left_motor_rot.getEncoder();
    }

    public RelativeEncoder getRightRotEncoder() {
        return right_motor_rot.getEncoder();
    }

    public void setAmperageLimit(int limit) {
        right_motor_ext.setSmartCurrentLimit((int)extension_amps.getDouble(limit));
        left_motor_ext.setSmartCurrentLimit((int)extension_amps.getDouble(limit));
    }

    public boolean checkIsFinished(double ext_pos, double rot_pos) {
        return(Math.abs(this.getLeftExtEncoder().getPosition() - ext_pos) <= Constants.ClimbSettings.TOLERANCE_LENGTH 
               && (Math.abs(this.getRightExtEncoder().getPosition() - ext_pos) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
               && (Math.abs(this.getLeftRotEncoder().getPosition() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROTATION)
               && (Math.abs(this.getRightRotEncoder().getPosition() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROTATION)
        );
    }
}

class ArmRotation {
// motors n stuff
    private CANSparkMax motor_rot;
    private SparkMaxPIDController pidController_rot;
    private SparkMaxLimitSwitch VerticalLimitSwitch;
// nts
    private NetworkTable network_table;
    private NetworkTableEntry nte_curr_pos_deg;
    private NetworkTableEntry nte_curr_pos_count;
    private NetworkTableEntry nte_des_pos_deg;
    private NetworkTableEntry nte_des_pos_count;
    private NetworkTableEntry nte_curr_current_amp;
    private NetworkTableEntry nte_curr_current_limit_amp;

    private final double CONVERSION_FACTOR;

    public ArmRotation(NetworkTable table, CANSparkMax motor_rot, boolean inverted) {
        
        // motors
        this.motor_rot = motor_rot;
        this.motor_rot.setInverted(inverted);

        VerticalLimitSwitch = motor_rot.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        this.pidController_rot = motor_rot.getPIDController();


        // NTs
        this.network_table = table;
        nte_curr_pos_deg = network_table.getEntry("Current Arm Rotation (degrees)");
        nte_curr_pos_deg.setDouble(0);
        nte_curr_pos_count = network_table.getEntry("Current Arm Rotation (counts)");
        nte_curr_pos_count.setDouble(0);
        nte_des_pos_deg = network_table.getEntry("Desired Arm Rotation (degrees)");
        nte_des_pos_deg.setDouble(0);
        nte_des_pos_count = network_table.getEntry("Desired Arm Rotation (counts)");
        nte_des_pos_count.setDouble(0);
        nte_curr_current_amp = network_table.getEntry("Current Amperage (amps)");
        nte_curr_current_amp.setDouble(0);
        nte_curr_current_limit_amp = network_table.getEntry("Amperage Limit (amps)");
        nte_curr_current_limit_amp.setDouble(0);

        CONVERSION_FACTOR = motor_rot.getEncoder().getPositionConversionFactor();
    }
    public void periodic() {
        double motor_curr_deg = motor_rot.getEncoder().getPosition();
        nte_curr_pos_deg.setDouble(motor_curr_deg);
        nte_curr_pos_count.setDouble(motor_curr_deg / CONVERSION_FACTOR);
        nte_curr_current_amp.setDouble(motor_rot.getOutputCurrent());
// current limit n stuff
        motor_rot.setSmartCurrentLimit((int) nte_curr_current_limit_amp.getDouble(5));
        
    }

    public void set(double degrees) {
        double counts = (42*174.9*degrees)/(360*20); //42 and 20 are the tooth pullies, 174.9 is the counts per rot and 360 is the degrees
        // 174.9 counter/rotation on motor
        // Motor is connected to a 20 tooth pulley that drives
        //    a 42 tooth pulley
        // changes the angle of the ? by this many degrees
        // l_rotator.getPIDController().setReference(degrees, ControlType.kPosition);
        // r_rotator.getPIDController().setReference(degrees, ControlType.kPosition);
        // left_Arm.set((float)counts);
        // right_Arm.set((float)counts);
        pidController_rot.setReference(counts, CANSparkMax.ControlType.kPosition);
        nte_des_pos_deg.setDouble(degrees);
        nte_des_pos_count.setDouble(counts);
    }
}

class ArmExtension {
    private NetworkTable network_table;
    private SparkMaxPIDController pidController_ext;
    private SparkMaxLimitSwitch ForwardLimitSwitch;
    private SparkMaxLimitSwitch ReverseLimitSwitch;
    private CANSparkMax motor_ext;
    private NetworkTableEntry nte_curr_pos_in;
    private NetworkTableEntry nte_curr_pos_count;
    private NetworkTableEntry nte_des_pos_in;
    private NetworkTableEntry nte_des_pos_count;
    private NetworkTableEntry nte_motor_amp_limit;
    private NetworkTableEntry nte_amps_now;
    private final int conversion_factor = (120/(15+(7/8))); //really dumb measured math probably not accurate
        // //double count = ((25.4*36*42)/(5*24))*inches; //25.4 is mm per inch, 36 is revs per 1 pev with gearbox, 42 is counts per rev and 5 is mm per 1 tooth
        // //   42 Counts per rev, but SmartMotion was showing rotations not counts
        // //   Connected to a 36:1 gearbox that could change
        // //   Gearbox connected to a 24 tooth pulley that is connected to a linear belt
        // //   Need to ask mechs to do analysis to convert 24 tooth pulley to linear distance;

    public ArmExtension(NetworkTable table, CANSparkMax motor_ext, boolean inverted){
        this.motor_ext = motor_ext;
        this.motor_ext.setInverted(inverted);

        ForwardLimitSwitch = motor_ext.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        ReverseLimitSwitch = motor_ext.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        this.pidController_ext = motor_ext.getPIDController(); 
        
        this.network_table = table;
        nte_curr_pos_in = this.network_table.getEntry("Current Arm Position (inches)");
        nte_curr_pos_in.setDouble(0);
        nte_curr_pos_count = this.network_table.getEntry("Current Arm Position (count)");
        nte_curr_pos_count.setDouble(0);
        nte_des_pos_in = this.network_table.getEntry("Desired Position (inches)");
        nte_des_pos_in.setDouble(0);
        nte_des_pos_count = this.network_table.getEntry("Desired Positon (count)");
        nte_des_pos_count.setDouble(0);
        nte_motor_amp_limit = this.network_table.getEntry("Motor Amp Limit");
        nte_motor_amp_limit.setDouble(0);
        nte_amps_now = this.network_table.getEntry("Amps Now");
        nte_amps_now.setDouble(0);
    }

    public void periodic(){
        nte_curr_pos_in.setDouble(motor_ext.getEncoder().getPosition()/conversion_factor);
        nte_curr_pos_count.setDouble(motor_ext.getEncoder().getPosition());
        motor_ext.setSmartCurrentLimit((int) nte_motor_amp_limit.getDouble(5));
        nte_amps_now.getDouble(motor_ext.getOutputCurrent());
    }
    public void set(double inches) {
        double count = inches*conversion_factor;
        pidController_ext.setReference(count, CANSparkMax.ControlType.kPosition);
        //motor_ext.getPIDController().setReference(count, CANSparkMax.ControlType.kPosition);
        nte_des_pos_in.setDouble(inches);
        nte_des_pos_count.setDouble(count);
    }
}