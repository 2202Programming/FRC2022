// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_Subsystem extends SubsystemBase {
  /** Creates a new Shooter_Subsystem. */
  public Shooter_Subsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed){
    //sets motor speed to go speed ft/s
  }

  //takes angle class (has vertical and horizontal)
  public void setAngle(Angle angle){
    //sets angles to angles designated by Angle class 
  }

  //Could return a Motor speed object
  public double getMotorSpeed(){
    //return speed of a motor in ft/s
  }

  public Angle getAngle(){
    //returns an angle class with the angle of the shooter 
  }

  public void shoot(){
    //shoots the ball
  }

  public class Angle(){
    double verticalAngle;
    double horizontalAngle; 

    public Angle(){

    }

    public double getVerticalAngle(){
      //gets vertical angle of shooter in degrees
    }

    public double getHorizontalAngle(){
      //gets horizontal angle of shooter in degrees
    }

    public void setVerticalAngle(double angle){
      //sets vertical angle to angle 
    }

    public void setHorizontalAngle(double angle){
      //sets horizontal angle to angle
    }


  }

}
