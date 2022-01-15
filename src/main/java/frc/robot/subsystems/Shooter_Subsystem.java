// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Shooter;
import frc.robot.util.PIDFController;

public class Shooter_Subsystem extends SubsystemBase {
  /** Creates a new Shooter_Subsystem. */
  static final int SLOT = 0;

  State  currentState = new State();
  TalonSRX flywheel = new TalonSRX(CAN.FLYWHEEL);
  TalonSRX flywheel1 = new TalonSRX(CAN.FLYWHEEL1);
  // RelativeEncoder flywheelEncoder = flywheel.getEncoder();
  // RelativeEncoder flywheelEncoder1 = flywheel1.getEncoder();
  // SparkMaxPIDController flywheelPID = flywheel.GetPid getPIDController();
  double speedGoal; 
 

  public Shooter_Subsystem() {
   
    // flywheelEncoder.setVelocityConversionFactor(1 / Shooter.CountsPerRev);
    //copy our pid values to the hardware 
    // flywheel.restoreFactoryDefaults();
    // flywheel1.restoreFactoryDefaults();
    // flywheel1.follow(flywheel, true);

    // Shooter.FlyWheelPID.copyTo(flywheelPID, SLOT);

    SmartDashboard.putNumber("Set Point 1", 0);
    SmartDashboard.putNumber("Set Point 2", 0);

    // flywheelEncoder.setVelocityConversionFactor(1 / Constants.NEO_COUNTS_PER_REVOLUTION);
  }

  public void display() {
    SmartDashboard.putNumber("P Gain", Shooter.FlyWheelPID.getP());
    SmartDashboard.putNumber("I Gain", Shooter.FlyWheelPID.getI());
    SmartDashboard.putNumber("D Gain", Shooter.FlyWheelPID.getD());
    SmartDashboard.putNumber("FF", Shooter.FlyWheelPID.getF());
  }
  
  public double clamp(String name) { 

    double setPoint = SmartDashboard.getNumber(name, 0);
    if(setPoint > 1) { setPoint = 1; }
    if(setPoint < -1) { setPoint = -1;}

    return setPoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO 1/15/2022 DPL - DONT set every frame, this is a can bus message
    // so you dont want more traffic.  Figure out a better way.  :)
    // flywheelPID.setReference(speedGoal, ControlType.kVelocity);
    // double xkP = SmartDashboard.getNumber("P Gain", Shooter.FlyWheelPID.getP());
    // double xkI = SmartDashboard.getNumber("I Gain", Shooter.FlyWheelPID.getI());
    // double xkD = SmartDashboard.getNumber("D Gain", Shooter.FlyWheelPID.getD());
    // double xkF = SmartDashboard.getNumber("FF", Shooter.FlyWheelPID.getF());

    // var xNewPid = new PIDFController(xkP, xkI, xkD, xkF);
    // if (!xNewPid.equals(Shooter.FlyWheelPID)) {
    //   Shooter.FlyWheelPID = xNewPid;
    //   Shooter.FlyWheelPID.copyTo(flywheelPID, SLOT);
    // }

    // flywheelPID.setReference(setPoint, ControlType.kVelocity);
    flywheel.set(ControlMode.PercentOutput, clamp("Set Point 1"));
    flywheel1.set(ControlMode.PercentOutput, -1 * clamp("Set Point 2"));

    SmartDashboard.putNumber("Encoder Velocity", flywheel.getSelectedSensorVelocity());// .getVelocity());
    SmartDashboard.putNumber("Encoder Velocity1", flywheel1.getSelectedSensorVelocity());
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
  // //takes angle class (has vertical and horizontal)
  // //ball comes out at requested angle
  // public void setAngle(Angle angle){
  //   //sets angles to angles designated by Angle class 
  // }

  // //Could return a Motor speed object
  // private double getMotorSpeed(){
  //   //return speed of a motor in ft/s
  // }

  // public Angle getAngle(){
  //   //returns an angle class with the angle of the shooter 
  // }

  // public double setTolerance(double tolerance){
  //   //tolerance of error
  // }

  // public void shoot(){
  //   //shoots the ball
  // }

  public boolean isReadyToShoot(){

    // TODO - some sort of test on speed and commanded
    getMotorSpeed();
    //returns wether the motors and angles are ready as set 
    return false;
  }
  // public boolean readyToShoot(){
  //   //returns wether the motors and angles are ready as set 
  // }

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

    // public State(){
  // public class Angle(){
  //   double verticalAngle;
  //   double horizontalAngle; 

  //   public Angle(){

  //   }

    public double getVerticalAngle(){
      //gets vertical angle of shooter in degrees
      return 0.0;
    }

    public double getHorizontalAngle(){
      //gets horizontal angle of shooter in degrees
      return 0.0;
    }
  //   public double getVerticalAngle(){
  //     //gets vertical angle of shooter in degrees
  //   }

  //   public double getHorizontalAngle(){
  //     //gets horizontal angle of shooter in degrees
  //   }

  //   public void setVerticalAngle(double angle){
  //     //sets vertical angle to angle 
  //   }

  //   public void setHorizontalAngle(double angle){
  //     //sets horizontal angle to angle
  //   }


  }

}