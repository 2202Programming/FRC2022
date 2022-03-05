package frc.robot.subsystems;

import java.util.concurrent.PriorityBlockingQueue;

import javax.print.attribute.standard.NumberUp;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
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
    private NetworkTableEntry nte_calibrate;
    // PIDSlot used
    int slot = 0;
    boolean calibrate = false;

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
        nte_calibrate = table.getEntry("calibrate");
        nte_calibrate.setBoolean(false);

        left_Arm_rot = new ArmRotation(table.getSubTable("left_arm_rotation"), left_motor_rot, false);
        right_Arm_rot = new ArmRotation(table.getSubTable("right_arm_rotation"), right_motor_rot, true);
        right_Arm_ext = new ArmExtension(table.getSubTable("right_arm_extension"), right_motor_ext, false);
        left_Arm_ext = new ArmExtension(table.getSubTable("left_arm_extension"), left_motor_ext, true);
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

    public void setRotation(double rotationDegrees) {
        if (left_Arm_rot != null)
            left_Arm_rot.set(rotationDegrees);
        if (right_Arm_rot != null)
            right_Arm_rot.set(rotationDegrees);
    }

    public void setSpeed(double left, double right) {
        // TODO implement arms with speed control
    }

    public void startCalibration() {
        left_Arm_ext.startCalibration();
        right_Arm_ext.startCalibration();
        System.out.println("started calibration");
    }

    @Override
    public void periodic() {
        if (nte_calibrate.getBoolean(calibrate) != calibrate && calibrate == false) {
            startCalibration();
            calibrate = true;
        }

        left_motor_rot.set(.5);
        left_Arm_ext.periodic();
        right_Arm_ext.periodic();
        left_Arm_rot.periodic();
        right_Arm_rot.periodic();
    }

    public double getLeftExtInches() {
        return left_Arm_ext.getInches();
    }

    public double getRightExtInches() {
        return right_Arm_ext.getInches();
    }

    public boolean isCalibrated() {
        return left_Arm_ext.isCalibrated() && right_Arm_ext.isCalibrated();
    }

    public double getLeftRotation() {
        if (left_Arm_rot == null)
            return 0;
        // return 0;
        return left_Arm_rot.getRotationDegrees();
    }

    public double getRightRotation() {
        if (right_Arm_rot == null)
            return 0;
        // return 0;
        return right_Arm_rot.getRotationDegrees();
    }

    public void setAmperageLimit(int limit) {
        // TODO: Create access methods in ArmExtension and ArmRotation objects, call
        // thise methods here
        right_motor_ext.setSmartCurrentLimit(limit);
        left_motor_ext.setSmartCurrentLimit(limit);
    }

    public boolean checkIsFinished(double ext_pos, double rot_pos) {
        return (Math.abs(this.getLeftExtInches() - ext_pos) <= Constants.ClimbSettings.TOLERANCE_LENGTH
                && (Math.abs(this.getRightExtInches() - ext_pos) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
                && (Math.abs(this.getLeftRotation() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROTATION)
                && (Math.abs(this.getRightRotation() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROTATION));
    }
}

class ArmRotation {
    // motors n stuff
    private CANSparkMax motor_rot;
    private SparkMaxPIDController pidController_rot;
    private SparkMaxLimitSwitch ForwardLimitSwitch;
    private SparkMaxLimitSwitch ReverseLimitSwitch;

    private RelativeEncoder encoder;

    // nts
    private NetworkTable network_table;
    private NetworkTableEntry nte_curr_pos_deg;
    private NetworkTableEntry nte_curr_pos_count;
    private NetworkTableEntry nte_des_pos_deg;
    private NetworkTableEntry nte_des_pos_count;

    private NetworkTableEntry nte_lower_limt;
    private NetworkTableEntry nte_upper_limt;

    // calibration
    private boolean calibrate = true;
    private NetworkTableEntry nte_calibrate;

    public ArmRotation(NetworkTable table, CANSparkMax motor_rot, boolean inverted) {

        // motors
        this.motor_rot = motor_rot;
        this.motor_rot.restoreFactoryDefaults();
        this.motor_rot.clearFaults();
        this.motor_rot.setInverted(inverted);
        setPercentOutput(0);

        ForwardLimitSwitch = motor_rot.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ReverseLimitSwitch = motor_rot.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
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

        nte_calibrate = this.network_table.getEntry("Calibrate");
        nte_calibrate.setBoolean(calibrate);

        nte_lower_limt = network_table.getEntry("Lower Limit");
        nte_lower_limt.setBoolean(false);
        nte_upper_limt = network_table.getEntry("Upper Limit");
        nte_upper_limt.setBoolean(false);

        encoder = motor_rot.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kQuadrature, 1);
        ClimbSettings.rotatePID.copyTo(motor_rot.getPIDController(), 0);
    }

    public void periodic() {
        
        double motor_curr_deg = encoder.getPosition();

        nte_curr_pos_count.setDouble(motor_curr_deg);
        nte_curr_pos_deg.setDouble(DegreesToEncoderCounts(motor_curr_deg));
        
        // current limit n stuff
        // motor_rot.setSmartCurrentLimit((int) nte_curr_current_limit_amp.getDouble(5));
        nte_lower_limt.setBoolean(ReverseLimitSwitch.isPressed());
        nte_upper_limt.setBoolean(ForwardLimitSwitch.isPressed());

        if (calibrate) {
            if (isLowerLimitHit()) {
                setPercentOutput(0);
                calibrate = false;
                nte_calibrate.setBoolean(calibrate);
                encoder.setPosition(0);
            } else {
                setPercentOutput(-.5);
            }
        }

    }

    public double DegreesToEncoderCounts(double degrees) {
        return (42 * 174.9 * degrees) / (360 * 20);
        // 42 and 20 are the tooth pullies, 174.9 is the counts per rot and 360 is the
        // degrees
        // 174.9 counter/rotation on motor
        // Motor is connected to a 20 tooth pulley that drives
        // a 42 tooth pulley
        // changes the angle of the ? by this many degrees
    }

    public double encoderCountsToDegrees(double counts) {
        return (counts * (360 * 20)) / (42 * 174.9);
    }

    public void set(double degrees) {
        // TODO: use conversion factor variable?
        double counts = DegreesToEncoderCounts(degrees);
        pidController_rot.setReference(counts, CANSparkMax.ControlType.kPosition);
        nte_des_pos_deg.setDouble(degrees);
        nte_des_pos_count.setDouble(counts);
    }

    public void setPercentOutput(double percentOutput) {
        motor_rot.set(percentOutput);
    }

    public boolean isLowerLimitHit() {
        return ReverseLimitSwitch.isPressed();
    }

    public double getRotationDegrees() {
        return encoderCountsToDegrees(encoder.getPosition());
    }
}

/**
 * Note: there are 3 limit switches --
 * 1 limit switch to check if extension is at top
 * 1 limit switch to check if extension is at bottom
 * 1 limit switch to check if rotation is vertical
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
    private NetworkTableEntry nte_calibrate;
    private NetworkTableEntry nte_lower_limt;
    private NetworkTableEntry nte_upper_limt;

    private final int conversion_factor = (90 / (14 + (3 / 4))); // really dumb measured math probably not accurate
    // //double count = ((25.4*36*42)/(5*24))*inches; //25.4 is mm per inch, 36 is
    // revs per 1 pev with gearbox, 42 is counts per rev and 5 is mm per 1 tooth
    // // 42 Counts per rev, but SmartMotion was showing rotations not counts
    // // Connected to a 36:1 gearbox that could change
    // // Gearbox connected to a 24 tooth pulley that is connected to a linear belt
    // // Need to ask mechs to do analysis to convert 24 tooth pulley to linear
    // distance;
    private boolean calibrate = false;

    public ArmExtension(NetworkTable table, CANSparkMax motor_ext, boolean inverted) {
        this.motor_ext = motor_ext;
        this.motor_ext.clearFaults();
        this.motor_ext.restoreFactoryDefaults();
        this.motor_ext.setInverted(inverted);
        this.motor_ext.setSmartCurrentLimit(40);

        ForwardLimitSwitch = motor_ext.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ReverseLimitSwitch = motor_ext.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
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
        nte_motor_amp_limit.setDouble(40);
        nte_amps_now = this.network_table.getEntry("Amps Now");
        nte_amps_now.setDouble(0);
        nte_calibrate = this.network_table.getEntry("Calibrate");
        nte_calibrate.setBoolean(calibrate);

        nte_lower_limt = this.network_table.getEntry("Lower Limit");
        nte_upper_limt = this.network_table.getEntry("Upper Limit");
        ClimbSettings.extendPID.copyTo(motor_ext.getPIDController(), 0);
    }

    public void startCalibration() {
        calibrate = true;
        nte_calibrate.setBoolean(calibrate);
    }

    public boolean isCalibrated() {
        return !calibrate;
    }

    public void periodic() {
        if (calibrate) {
            if (isLowerLimitHit()) {
                setPercentOutput(0);
                calibrate = false;
                nte_calibrate.setBoolean(calibrate);
                motor_ext.getEncoder().setPosition(0);
            } else {
                setPercentOutput(-.2);
            }
        }
        setPercentOutput(0);
        nte_lower_limt.setBoolean(ReverseLimitSwitch.isPressed());
        nte_upper_limt.setBoolean(ForwardLimitSwitch.isPressed());

        nte_curr_pos_in.setDouble(motor_ext.getEncoder().getPosition() / conversion_factor);
        nte_curr_pos_count.setDouble(motor_ext.getEncoder().getPosition());
        motor_ext.setSmartCurrentLimit((int) nte_motor_amp_limit.getDouble(5));
        nte_amps_now.getDouble(motor_ext.getOutputCurrent());
    }

    public void set(double inches) {
        if (calibrate)
            return;
        double count = inches * conversion_factor;
        pidController_ext.setReference(count, CANSparkMax.ControlType.kPosition);
        // motor_ext.getPIDController().setReference(count,
        // CANSparkMax.ControlType.kPosition);
        nte_des_pos_in.setDouble(inches);
        nte_des_pos_count.setDouble(count);
    }

    public void setPercentOutput(double percentOutput) {
        motor_ext.set(percentOutput);
    }

    public boolean isLowerLimitHit() {
        return ReverseLimitSwitch.isPressed();
    }

    public double getInches() {
        return this.motor_ext.getEncoder().getPosition() * conversion_factor;
    }
}