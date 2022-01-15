// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Shooter;

public class Shooter_Subsystem extends SubsystemBase {
  /** Creates a new Shooter_Subsystem. */
  static final int SLOT = 0;

  State  currentState = new State();
  CANSparkMax flywheel = new CANSparkMax(CAN.FLYWHEEL, MotorType.kBrushless);
  RelativeEncoder flywheelEncoder = flywheel.getEncoder();
  SparkMaxPIDController flywheelPID = flywheel.getPIDController();
  double speedGoal; 
 

  public Shooter_Subsystem() {
   
    flywheelEncoder.setVelocityConversionFactor(1 / Shooter.CountsPerRev);
    //copy our pid values to the hardware 
    Shooter.FlyWheelPID.copyTo(flywheelPID, SLOT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO 1/15/2022 DPL - DON'T set every frame, this is a can bus message
    // so you don't want more traffic.  Figure out a better way.  :)
    flywheelPID.setReference(speedGoal, ControlType.kVelocity);
  }

  //ball should come out at this speed
  public void setSpeed(double speed /*in rpm*/){
    //turns this into a motor speed
    //spin motor at rpm
    speed = speed / 60; //convert rpm to rps 
    speedGoal = speed; 
  }

  //takes angle class (has vertical and horizontal)
  //ball comes out at requested angle
  public void setState(State cmdState){
    //sets angles to angles designated by Angle class 
  }

  //Could return a Motor speed object
  private double getMotorSpeed(){
    //return speed of a motor in ft/s
    return 0.0;
  }

  public State getState() {
    //returns an angle class with the angle of the shooter 
    return this.currentState;
  }

  public void setTolerance(double tolerance){
    //tolerance of error

  }

  public void shoot(){
    //shoots the ball
  }

  public boolean isReadyToShoot(){

    // TODO - some sort of test on speed and commanded
    getMotorSpeed();
    //returns wether the motors and angles are ready as set 
    return false;
  }

  /*
  * Might want to make "grand setter command" that sets all relevant values
   * TODO DPL - changed name to State because it may hold more than Angle
   * 
  */

  public class State {
    double verticalAngle;
    double horizontalAngle; 
    double angleTolerance ;   //TODO - more possible examples of state 
    double speedTolerance ;

    public State(){

    }

    public double getVerticalAngle(){
      //gets vertical angle of shooter in degrees
      return 0.0;
    }

    public double getHorizontalAngle(){
      //gets horizontal angle of shooter in degrees
      return 0.0;
    }

    public void setVerticalAngle(double angle){
      //sets vertical angle to angle 
    }

    public void setHorizontalAngle(double angle){
      //sets horizontal angle to angle
    }


  }

}