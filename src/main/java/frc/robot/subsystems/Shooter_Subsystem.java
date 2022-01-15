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
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Shooter_Subsystem extends SubsystemBase {
  /** Creates a new Shooter_Subsystem. */
  CANSparkMax flywheel = new CANSparkMax(CAN.FLYWHEEL, MotorType.kBrushless);
  RelativeEncoder flywheelEncoder = flywheel.getEncoder();
  SparkMaxPIDController flywheelPID = flywheel.getPIDController();
  double speedGoal; 
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;

  public Shooter_Subsystem() {
    flywheelPID.setP(kP);
    flywheelPID.setI(kI);
    flywheelPID.setD(kD);
    flywheelEncoder.setVelocityConversionFactor(1 / Constants.NEO_COUNTS_PER_REVOLUTION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
  public void setAngle(Angle angle){
    //sets angles to angles designated by Angle class 
  }

  //Could return a Motor speed object
  private double getMotorSpeed(){
    //return speed of a motor in ft/s
    return 0;
  }

  public Angle getAngle(){
    //returns an angle class with the angle of the shooter 
    return new Angle();
  }

  public double setTolerance(double tolerance){
    //tolerance of error
    return 0;
  }

  public void shoot(){
    //shoots the ball
  }

  public boolean readyToShoot(){
    //returns wether the motors and angles are ready as set 
    return false;
  }

  /*
  * Might want to make "grand setter command" that sets all relevant values
  */

  public class Angle {
    double verticalAngle;
    double horizontalAngle; 

    public Angle(){

    }

    public double getVerticalAngle(){
      //gets vertical angle of shooter in degrees
      return 0;
    }

    public double getHorizontalAngle(){
      //gets horizontal angle of shooter in degrees
      return 0;
    }

    public void setVerticalAngle(double angle){
      //sets vertical angle to angle 
    }

    public void setHorizontalAngle(double angle){
      //sets horizontal angle to angle
    }


  }

}
