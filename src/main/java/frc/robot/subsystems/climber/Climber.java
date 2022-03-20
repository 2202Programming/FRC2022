package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
    private NetworkTableEntry nte_sync_arms;

    // command fixes
    boolean outerLoopEnabled = false;   // use sw position pids to control velocity

    // Compensation "syncArms" 
    boolean syncArmsEnabled;                // when true uses differential pos err to compensate
    PIDController extCompPID = new PIDController(2.0, 0, 0);    // input [in], output [in/s] Kp=[(in/s)/in-err]  
    PIDController rotCompPID = new PIDController(1.00, 0, 0);   // input [deg], output [deg/s] Kp=[(deg/s)/deg-err]
    double rot_compensation = 0.0;                              // [deg/s] from rotCompPID
    double ext_compensation = 0.0;                              // [in/s] from extCompPID
    
    // postion goals - software outer loops for position
    PIDController rotPosL = new PIDController(0.5, 0.0, 0.0); // in degs-err out: vel deg/s
    PIDController rotPosR = new PIDController(0.5, 0.0, 0.0); // in degs-err out: vel deg/s
    PIDController extPosL = new PIDController(5.0, 0.0, 0.0); // in inch-err out vel in/s
    PIDController extPosR = new PIDController(5.0, 0.0, 0.0); // in inch-err out vel in/s

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
        nte_sync_arms = table.getEntry("syncArms");
        nte_sync_arms.setBoolean(syncArmsEnabled);

        left_Arm_rot = new ArmRotation(table.getSubTable("left_arm_rotation"), left_motor_rot, true, 0.9);
        right_Arm_rot = new ArmRotation(table.getSubTable("right_arm_rotation"), right_motor_rot, false, 0.5);
        right_Arm_ext = new ArmExtension(table.getSubTable("right_arm_extension"), right_motor_ext, false);
        left_Arm_ext = new ArmExtension(table.getSubTable("left_arm_extension"), left_motor_ext, true);

        //Outer loop position tolerance
        rotPosL.setTolerance(ClimbSettings.TOLERANCE_ROT, ClimbSettings.TOLERANCE_ROT_RATE);
        rotPosR.setTolerance(ClimbSettings.TOLERANCE_ROT, ClimbSettings.TOLERANCE_ROT_RATE);
        extPosL.setTolerance(ClimbSettings.TOLERANCE_EXT, ClimbSettings.TOLERANCE_EXT_VEL);
        extPosR.setTolerance(ClimbSettings.TOLERANCE_EXT, ClimbSettings.TOLERANCE_EXT_VEL);

        setArmSync(false);
        setOuterLoop(false);
        setStartingPos();
        // finish hardware limits
        setAmperageExtLimit(ClimbSettings.MAX_EXT_AMPS);
        setAmperageRotLimit(ClimbSettings.MAX_ROT_AMPS);
    }

    public void setStartingPos() {
        // approx centered
        left_Arm_ext.setEncoderPos(0.0);
        right_Arm_ext.setEncoderPos(0.0);

        // arms vertical
        left_Arm_rot.setEncoderPos(0.0);
        right_Arm_rot.setEncoderPos(0.0);

        extCompPID.reset();
        rotCompPID.reset();
        extCompPID.setSetpoint(0.0);
        rotCompPID.setSetpoint(0.0);

        //clear all position pids
        extPosL.reset();
        extPosR.reset();
        rotPosL.reset();
        rotPosR.reset();
        //clear all position pids
        extPosL.setSetpoint(0.0);
        extPosR.setSetpoint(0.0);
        rotPosL.setSetpoint(0.0);
        rotPosR.setSetpoint(0.0);
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
        extPosL.setSetpoint(inches);
        extPosR.setSetpoint(inches);
    }

    public void setRotation(double rotationDegrees) {
        rotPosL.setSetpoint(rotationDegrees);
        rotPosR.setSetpoint(rotationDegrees);
    }

    /**
     * 
     * @param ext_spd [in/s]
     */
    public void setExtSpeed(double ext_spd) {
        setExtSpeed(ext_spd, ext_spd);
    }

    public void setExtSpeed(double ext_spdlt, double ext_spdrt) {
        //split the sync comp and remove/add a bit
        double comp = (syncArmsEnabled) ? ext_compensation/2.0 : 0.0;
        left_Arm_ext.setSpeed(ext_spdlt - comp);
        right_Arm_ext.setSpeed(ext_spdrt + comp);
    }

    /**
     * 
     * @param rot_spd [deg/s]
     */
    public void setRotSpeed(double rot_spd) {
        setRotSpeed(rot_spd, rot_spd);
    }

    public void setRotSpeed(double rot_spd_lt, double rot_spd_rt) {
        double comp = (syncArmsEnabled) ? rot_compensation/2.0 : 0.0;
        left_Arm_rot.setRotRate(rot_spd_lt - comp);
        right_Arm_rot.setRotRate(rot_spd_rt + comp);
    }

    public void setArmSync(boolean sync) {
        syncArmsEnabled = sync;
        nte_sync_arms.setBoolean(syncArmsEnabled);
    }


    public void hold() {
        //hardware stop
        setExtSpeed(0.0);
        setRotSpeed(0.0);

        //clear sw pid states
        extPosR.reset();
        extPosL.reset();
        rotPosL.reset();
        rotPosR.reset();

        //command where we are, update setpoints with current position
        extPosL.setSetpoint(left_Arm_ext.getInches());
        extPosR.setSetpoint(right_Arm_ext.getInches());
        rotPosL.setSetpoint(left_Arm_rot.getRotationDegrees());
        rotPosR.setSetpoint(right_Arm_rot.getRotationDegrees());

        //outerloop & syncArms as controled by command
    }


    @Override
    public void periodic() {
        // control left with PIDs, rt will follow
        double ext_velL = extPosL.calculate(getLeftExtInches());
        double rot_velL = rotPosL.calculate(getLeftRotation());
        double ext_velR = extPosR.calculate(getRightExtInches());
        double rot_velR = rotPosR.calculate(getRightRotation());

        ext_velL = MathUtil.clamp(ext_velL, -ClimbSettings.MAX_VELOCITY_EXT, ClimbSettings.MAX_VELOCITY_EXT);
        rot_velL = MathUtil.clamp(rot_velL, -ClimbSettings.MAX_VELOCITY_ROT, ClimbSettings.MAX_VELOCITY_ROT);
        ext_velR = MathUtil.clamp(ext_velR, -ClimbSettings.MAX_VELOCITY_EXT, ClimbSettings.MAX_VELOCITY_EXT);
        rot_velR = MathUtil.clamp(rot_velR, -ClimbSettings.MAX_VELOCITY_ROT, ClimbSettings.MAX_VELOCITY_ROT);

        //c
        ext_compensation = 0.0;
        ext_compensation = 0.0;
        if (syncArmsEnabled) {
            extCompPID.setSetpoint(getLeftExtInches());
            rotCompPID.setSetpoint(getLeftRotation());
            ext_compensation = extCompPID.calculate(getRightExtInches());
            rot_compensation = rotCompPID.calculate(getRightRotation());
        }
      
        // output new speed settings
        if (outerLoopEnabled) { 
            setExtSpeed(ext_velL, ext_velR);
            setRotSpeed(rot_velL, rot_velR);
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

    public double getLeftRotation() {
        return left_Arm_rot.getRotationDegrees();
    }

    public double getRightRotation() {
        return right_Arm_rot.getRotationDegrees();
    }

    public void setAmperageExtLimit(int limit) {
        right_motor_ext.setSmartCurrentLimit(limit);
        left_motor_ext.setSmartCurrentLimit(limit);
    }

    public void setAmperageRotLimit(int limit) {
        // brushless only
        //right_motor_rot.setSmartCurrentLimit(limit);
        //left_motor_rot.setSmartCurrentLimit(limit);
        // brushed
        right_motor_rot.setSecondaryCurrentLimit(limit);
        left_motor_rot.setSecondaryCurrentLimit(limit);
    }

    public void setOuterLoop(boolean enable) {
        outerLoopEnabled = enable;
    }

    /**
     * outerLoopDone checks for rotation,extension, and combined when using
     * the software pids for positon control.
     * @return
     */
    public boolean outerLoopExtDone() {
        return extPosL.atSetpoint() && extPosR.atSetpoint();
    }
    public boolean outerLoopRotDone() {
        return rotPosL.atSetpoint() && rotPosR.atSetpoint();
    }
    public boolean outerLoopDone() {
        return outerLoopExtDone() && outerLoopRotDone();
    }

    /**
     *   Note - this doesn't check the velocity of the actuatorators.
     *   Consider using the outerLoop softare pid tests.
     *
     * @param ext_pos   target extension
     * @param rot_pos   target rotation
     * @return
     */
    public boolean checkIsFinished(double ext_pos, double rot_pos) {
        boolean extDone = (Math.abs(this.getLeftExtInches() - ext_pos) <= ClimbSettings.TOLERANCE_EXT)
                && (Math.abs(this.getRightExtInches() - ext_pos) <= ClimbSettings.TOLERANCE_EXT);
        boolean rotDone = 
                (Math.abs(this.getLeftRotation() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROT)
                && (Math.abs(this.getRightRotation() - rot_pos) <= Constants.ClimbSettings.TOLERANCE_ROT);
        return (extDone && rotDone);
    }

    /**
     * accessors for rotation and extension objects - used for testing
     * 
     * @return
     */
    public ArmRotation getLeftArmRotation() {
        return left_Arm_rot;
    }

    public ArmRotation getRightArmRotation() {
        return right_Arm_rot;
    }

    public ArmExtension getLeftArmExtension() {
        return left_Arm_ext;
    }

    public ArmExtension getRightArmExtension() {
        return right_Arm_ext;
    }

    @Deprecated
    public void setPercentOutputRot(double pct_l, double pct_r) {
        left_Arm_rot.setPercentOutput(pct_l);
        right_Arm_rot.setPercentOutput(pct_r);
    }
}