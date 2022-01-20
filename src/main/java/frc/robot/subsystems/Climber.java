package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class Climber {
    //raise/lower
    private CANSparkMax raise1; 
    private CANSparkMax lower1; 
    private CANSparkMax raise2; 
    private CANSparkMax lower2; 

    //actuation (rotate?)
    private CANSparkMax angle1;
    private CANSparkMax angle2;


    public Climber() {
        raise1 = new CANSparkMax(Constants.CAN.RAISE1);
        lower1 = new CANSparkMax(Constants.CAN.LOWER1);
        raise2 = new CANSparkMax(Constants.CAN.RAISE2);
        lower2 = new CANSparkMax(Constants.CAN.LOWER2);

        angle1 = new CANSparkMax(Constants.CAN.ANGLE1);
        angle2 = new CANSparkMax(Constants.CAN.ANGLE2);

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

    //for raising/lower
    public void raise(double inches){
        //negative values lower
        //assumed vertical 
    }

    public void angle(double degrees){
        //changes the angle of the ? by this many degrees
    }    

}
