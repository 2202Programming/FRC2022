package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ClimbSettings;
import frc.robot.subsystems.climber.ArmRotation;
import frc.robot.subsystems.climber.ArmExtension;

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

        left_Arm_rot = new ArmRotation(table.getSubTable("left_arm_rotation"), left_motor_rot, true);
        right_Arm_rot = new ArmRotation(table.getSubTable("right_arm_rotation"), right_motor_rot, false);
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
        System.out.println("started calibration");
        left_Arm_ext.startCalibration();
        right_Arm_ext.startCalibration();
        left_Arm_rot.startCalibration();
        right_Arm_rot.startCalibration();
    }

    @Override
    public void periodic() {
        if (nte_calibrate.getBoolean(calibrate) != calibrate && calibrate == false) {
            startCalibration();
            calibrate = true;
        }
        
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

    public ArmRotation getLeftArmRotation(){
        return left_Arm_rot;
    }

    public ArmRotation getRightArmRotation(){
        return right_Arm_rot;
    }

    public ArmExtension getLeftArmExtension(){
        return left_Arm_ext;
    }

    public ArmExtension getRightArmExtension(){
        return right_Arm_ext;
    }
}