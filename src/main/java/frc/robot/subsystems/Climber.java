package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.CAN;
import static frc.robot.Constants.ClimbSettings;

public class Climber {
    // PIDSlot used
    int slot = 0;

    // raise/lower controllers
    private CANSparkMax left_extender= new CANSparkMax(CAN.CMB_LEFT_Extend, MotorType.kBrushless);
    private CANSparkMax right_extender = new CANSparkMax(CAN.CMB_RIGHT_Extend, MotorType.kBrushless);

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
        ClimbSettings.innerPID.copyTo(left_extender.getPIDController(), slot);
        ClimbSettings.innerPID.copyTo(right_extender.getPIDController(), slot);


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
    // @param inches from extender current position
    public void setExtension(double inches) {
        float count =  (float) inches; // convert command inches to encoder counts
        SoftLimitDirection direction = null ;
        double speed = 0 ;
        // if inches > 0 then the extender moves forward at 25% speed
        if (inches > 0){
            direction = SoftLimitDirection.kForward;
            speed = 0.25;
        }
        // if inches < 0 then the extender moves backwards at 25% speed
        else if (inches < 0){
            direction = SoftLimitDirection.kReverse;
            speed = -0.25;
        }
        // if inches = 0 then we don't need to move
        else {
            return;
        }


        // TODO command left_extender and right_extender to move to specified encoder counts
        left_extender.setSoftLimit(direction, count);
        left_extender.set(speed);
        right_extender.setSoftLimit(direction, count);
        right_extender.set(speed);
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
        // changes the angle of the ? by this many degrees
        // l_rotator.getPIDController().setReference(degrees, ControlType.kPosition);
        // r_rotator.getPIDController().setReference(degrees, ControlType.kPosition);

    }

}
