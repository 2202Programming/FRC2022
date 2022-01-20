package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.CAN;
import static frc.robot.Constants.ClimbSettings;

public class Climber {
    // PIDSlot used
    int slot = 0;

    //raise/lower controllers
    private CANSparkMax li_extender = new CANSparkMax(CAN.CMB_LI_Extend, MotorType.kBrushless);
    private CANSparkMax lo_extender = new CANSparkMax(CAN.CMB_LO_Extend, MotorType.kBrushless);
    private CANSparkMax ri_extender = new CANSparkMax(CAN.CMB_RI_Extend, MotorType.kBrushless);
    private CANSparkMax ro_extender = new CANSparkMax(CAN.CMB_RO_Extend, MotorType.kBrushless);

    //rotation arm controller (outer arms rotate)
    private CANSparkMax l_rotator = new CANSparkMax(CAN.CMB_L_Rotate, MotorType.kBrushless);
    private CANSparkMax r_rotator = new CANSparkMax(CAN.CMB_R_Rotate, MotorType.kBrushless);

  
    public Climber() {
        // TODO Set the control type and PID settings - most likely position control
        
        //Copy the PID setting down to the hardware
        // Rotation PIDS
        ClimbSettings.armPID.copyTo(l_rotator.getPIDController(), slot);
        ClimbSettings.armPID.copyTo(r_rotator.getPIDController(), slot);
        // Arm extension PIDS
        ClimbSettings.innerPID.copyTo(li_extender.getPIDController(), slot);
        ClimbSettings.innerPID.copyTo(ri_extender.getPIDController(), slot);
        ClimbSettings.outerPID.copyTo(lo_extender.getPIDController(), slot);
        ClimbSettings.outerPID.copyTo(ro_extender.getPIDController(), slot);

    }

    public boolean readyToClimb(){
        //returns if robot is in the right position, and all motors are in place
        return false;
    }

    public String rung(){
        return "blank";
    }

    public boolean readyToTraverse(){
        return false;
    }

    /**
     * Left and Right arms are controlled in pairs
     *      L/R Inner arms move together
     *      L/R Outer arms move together
     * @param inches from retracted position
     */
    //for raising/lower
    public void setInnerExtension(double inches){
        //negative values lower
        //assumed vertical 
    }
    
    /**
     * Left and Right arms are controlled in pairs
     *      L/R Outer arms move together
     * @param inches from retracted position
     */
    public void setOuterExtension(double inches){
    //negative values lower
    //assumed vertical 
}
    /**
     * Outer L/R arms rotate together
     * 
     * @param degrees  +/- degrees from vertical
     */
    public void setRotation(double degrees){
        //changes the angle of the ? by this many degrees
        l_rotator.getPIDController().setReference(degrees, ControlType.kPosition);
        r_rotator.getPIDController().setReference(degrees, ControlType.kPosition);

    }    

}
