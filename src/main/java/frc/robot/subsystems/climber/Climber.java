package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Climber extends SubsystemBase {
    // NTs
    private NetworkTable table;
    private NetworkTableEntry nte_sync_arms;

    //sync values
    boolean sync_arms = false;    // when true uses diff_x_errs to synchronize
    double diff_rot_err = 0.0;
    double diff_ext_err = 0.0;
    double ext_compensation = 0.0;                        // [in/s] from extPID
    PIDController extPID = new PIDController(.1, 0, 0);   // input [in], output [in/s] Kp=[(in/s)/in-err]
        double rot_compensation = 0.0;                    // [deg/s] from rotPID
    PIDController rotPID = new PIDController(.1, 0, 0);   // input [deg], output [deg/s] Kp=[(deg/s)/deg-err]
    
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
        nte_sync_arms = table.getEntry("sync_arms");
        nte_sync_arms.setBoolean(sync_arms);

        left_Arm_rot = new ArmRotation(table.getSubTable("left_arm_rotation"), left_motor_rot, true);
        right_Arm_rot = new ArmRotation(table.getSubTable("right_arm_rotation"), right_motor_rot, false);
        right_Arm_ext = new ArmExtension(table.getSubTable("right_arm_extension"), right_motor_ext, false);
        left_Arm_ext = new ArmExtension(table.getSubTable("left_arm_extension"), left_motor_ext, true);

        //TODO -pull from constants  setAmperageLimit(limit);

        setStartingPos();
    }

    public void setStartingPos() {
        //approx centered
        left_Arm_ext.setEncoderPos(0.0);
        right_Arm_ext.setEncoderPos(0.0);
        
        // arms vertical
        left_Arm_rot.setEncoderPos(0.0);
        right_Arm_rot.setEncoderPos(0.0);
        
        //always zero setpoint because we want no difference between L and R
        extPID.setSetpoint(0.0);  
        rotPID.setSetpoint(0.0);
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
        left_Arm_ext.setInches(inches);
        right_Arm_ext.setInches(inches);
    }

    public void setRotation(double rotationDegrees) {
        left_Arm_rot.set(rotationDegrees);
        right_Arm_rot.set(rotationDegrees);
    }

    /**
     * 
     * @param spd [in/s]
     */
    public void setExtSpeed(double spd) { setExtSpeed(spd, spd);}
    public void setExtSpeed(double v_lt, double v_rt) {
        double rt_comp = (sync_arms) ? ext_compensation : 0.0;
        left_Arm_ext.setSpeed(v_lt);
        right_Arm_ext.setSpeed(v_rt + rt_comp);
    }
        

    /**
     * 
     * @param rot_spd [deg/s]
     */
    public void setRotSpeed(double rot_spd) { setExtSpeed(rot_spd, rot_spd);}
    public void setRotSpeed(double rot_spd_lt, double rot_spd_rt) {
        double rt_comp = (sync_arms) ? rot_compensation : 0.0 ;
        left_Arm_rot.setRotRate(rot_spd_lt);
        right_Arm_rot.setRotRate(rot_spd_rt + rt_comp);
    }

    public void setArmSync(boolean sync) {
        sync_arms = sync;
        nte_sync_arms.setBoolean(sync_arms);
    }

    @Override
    public void periodic() {
        
        left_Arm_ext.periodic();
        right_Arm_ext.periodic();
        left_Arm_rot.periodic();
        right_Arm_rot.periodic();

        // form position error (left - right) to compensate commanded velocity
        diff_rot_err = left_Arm_rot.getRotationDegrees() - right_Arm_rot.getRotationDegrees();
        diff_ext_err = left_Arm_ext.getInches() - right_Arm_ext.getInches();

        ext_compensation = extPID.calculate(diff_ext_err);
        rot_compensation = extPID.calculate(diff_rot_err);
    }

    public double getLeftExtInches() {
        return left_Arm_ext.getInches();
    }

    public double getRightExtInches() {
        return right_Arm_ext.getInches();
    }

    public double getLeftRotation() {
        if (left_Arm_rot == null)
            return 0;
        return left_Arm_rot.getRotationDegrees();
    }

    public double getRightRotation() {
        if (right_Arm_rot == null)
            return 0;
        return right_Arm_rot.getRotationDegrees();
    }

    public void setAmperageLimit(int limit) {
        right_motor_ext.setSmartCurrentLimit(limit);
        left_motor_ext.setSmartCurrentLimit(limit);
    }

    public boolean checkIsFinished(double ext_pos, double rot_pos) {
        return (Math.abs(this.getLeftExtInches() - ext_pos) <= Constants.ClimbSettings.TOLERANCE_LENGTH
                && (Math.abs(this.getRightExtInches() - ext_pos) <= Constants.ClimbSettings.TOLERANCE_LENGTH)
                && (Math.abs(this.getLeftRotation() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROTATION)
                && (Math.abs(this.getRightRotation() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROTATION));
    }

    /**
     * accessors for rotation and extension objects - used for testing
     * @return
     */
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