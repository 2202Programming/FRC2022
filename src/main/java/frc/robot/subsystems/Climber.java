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

 
    

    
    private CANSparkMax left_motor_rot = new CANSparkMax(CAN.CMB_LEFT_Rotate, MotorType.kBrushed);
    private CANSparkMax right_motor_rot = new CANSparkMax(CAN.CMB_RIGHT_Rotate, MotorType.kBrushed);
    private CANSparkMax left_motor_ext = new CANSparkMax(CAN.CMB_LEFT_Extend, MotorType.kBrushless);
    private CANSparkMax right_motor_ext = new CANSparkMax(CAN.CMB_RIGHT_Extend, MotorType.kBrushless);


    private ArmExtension left_Arm_ext;
    private ArmExtension right_Arm_ext;
    private ArmRotation left_Arm_rot;
    private ArmRotation right_Arm_rot;


    public Climber() {
        table = NetworkTableInstance.getDefault().getTable("Climber");


        left_Arm_rot = new ArmRotation(table.getSubTable("left_arm_rotation"), left_motor_rot, false);
        right_Arm_rot = new ArmRotation(table.getSubTable("right_arm_rotation"), right_motor_rot, true);
        right_Arm_ext = new ArmExtension(table.getSubTable("right_arm_extension"), right_motor_ext, false);
        left_Arm_ext =new ArmExtension(table.getSubTable("left_arm_extension"), left_motor_ext, true);

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

        left_Arm_ext.set(inches);
        right_Arm_ext.set(inches);

    }

    public void setRotation(double rotationDegrees){
        left_Arm_rot.set(rotationDegrees);
        right_Arm_rot.set(rotationDegrees);
    }


    public void setSpeed(double left, double right) 
    {
        //TODO implement arms with speed control
    }

    
    
    public void periodic() {
        left_Arm_ext.periodic();
        right_Arm_ext.periodic();
        left_Arm_rot.periodic();
        right_Arm_rot.periodic();
    }

    public double getLeftExtInches(){
        return left_Arm_ext.getInches();
    }

    public double getRightExtInches(){
        return right_Arm_ext.getInches();
    }

    public double getLeftRotation(){
        return left_Arm_rot.getRotationDegrees();
    }

    public double getRightRotation(){
        return right_Arm_rot.getRotationDegrees();
    }

    public void setAmperageLimit(int limit) {
        // TODO: Create access methods in ArmExtension and ArmRotation objects, call thise methods here
        right_motor_ext.setSmartCurrentLimit(limit);
        left_motor_ext.setSmartCurrentLimit(limit);
    }

    public boolean checkIsFinished(double ext_pos, double rot_pos) {
        return(Math.abs(this.getLeftExtInches() - ext_pos) <= Constants.ClimbSettings.TOLERANCE_LENGTH 
               && (Math.abs(this.getRightExtInches() - ext_pos) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
               && (Math.abs(this.getLeftRotation() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROTATION)
               && (Math.abs(this.getRightRotation() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROTATION)
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

    private final double CONVERSION_FACTOR; // TODO: Does this need to be set here? Why are we reading the motor conversion factor and not setting it?

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
        ClimbSettings.rotatePID.copyTo(motor_rot.getPIDController(), 0);
    }
    public void periodic() {
        double motor_curr_deg = motor_rot.getEncoder().getPosition();
        nte_curr_pos_deg.setDouble(motor_curr_deg);
        nte_curr_pos_count.setDouble(motor_curr_deg / CONVERSION_FACTOR);
        nte_curr_current_amp.setDouble(motor_rot.getOutputCurrent());
        // current limit n stuff
        motor_rot.setSmartCurrentLimit((int) nte_curr_current_limit_amp.getDouble(5));
        
    }

    public double DegreesToEncoderCounts(double degrees){
        return (42*174.9*degrees)/(360*20);
         //42 and 20 are the tooth pullies, 174.9 is the counts per rot and 360 is the degrees
        // 174.9 counter/rotation on motor
        // Motor is connected to a 20 tooth pulley that drives
        //    a 42 tooth pulley
        // changes the angle of the ? by this many degrees
    }

    public double encoderCountsToDegrees(double counts){
        return (counts*(360*20))/(42*174.9);
    }

    public void set(double degrees) {
        // TODO: use conversion factor variable?
        double counts = DegreesToEncoderCounts(degrees);
        pidController_rot.setReference(counts, CANSparkMax.ControlType.kPosition);
        nte_des_pos_deg.setDouble(degrees);
        nte_des_pos_count.setDouble(counts);
    }
    public double getRotationDegrees(){
        return encoderCountsToDegrees(motor_rot.getEncoder().getPosition());
    }
}
 /**
     * Note: there are 3 limit switches --
     *  1 limit switch to check if extension is at top
     *  1 limit switch to check if extension is at bottom
     *  1 limit switch to check if rotation is vertical
     */

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
        this.motor_ext.clearFaults();
        this.motor_ext.restoreFactoryDefaults();
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
        ClimbSettings.extendPID.copyTo(motor_ext.getPIDController(), 0);
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

    public double getInches(){
        return this.motor_ext.getEncoder().getPosition()*conversion_factor;
    }
}