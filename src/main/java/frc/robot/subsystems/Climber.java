package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Encoder;

import static frc.robot.Constants.CAN;
import static frc.robot.Constants.ClimbSettings;

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
    private Relay left_Relay_rot;
    private Relay right_Relay_rot;
    private Encoder left_Encoder_rot;
    private Encoder right_Encoder_rot;
    private double left_current_angle;
    private double right_current_angle;

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
        

        left_Relay_rot = new Relay(0);
        right_Relay_rot = new Relay(0);
        left_Encoder_rot = new Encoder(0,1); // Initializes an encoder on DIO pins 0 and 1
        right_Encoder_rot = new Encoder(2,3); // Intializes an encoder on DIO pins 2 and 3
        left_Encoder_rot.reset();
        right_Encoder_rot.reset();
        left_current_angle = 0;
        right_current_angle = 0;
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
        double left_Encoder_distance = left_Encoder_rot.getDistance();
        double right_Encoder_distance = right_Encoder_rot.getDistance();
        double closeEnough = 2; //The current distance is between 2 degrees of the desired distance
        if (Math.abs(degrees - left_Encoder_distance)<closeEnough){
            left_Relay_rot.set(Relay.Value.kOff);
        } else {
            if (left_Encoder_distance < degrees){
                left_Relay_rot.set(Relay.Value.kForward);
            } else {
                left_Relay_rot.set(Relay.Value.kReverse);
            }
        }
        if (Math.abs(degrees - right_Encoder_distance)<closeEnough){
            right_Relay_rot.set(Relay.Value.kOff);
        } else {
            if (right_Encoder_distance < degrees){
                right_Relay_rot.set(Relay.Value.kForward);
            } else {
                right_Relay_rot.set(Relay.Value.kReverse);
            }
        }
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
