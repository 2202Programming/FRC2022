// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.Constants.DigitalIO;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.MagazineSettings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN;

public class Magazine_Subsystem extends SubsystemBase {
  //slot to use on controllers
  int slot = 0;
  
  /** Creates a new Magazine2. */
  private TalonSRX top_wheel;
  private CANSparkMax r_belt = new CANSparkMax(CAN.MAG_R_BELT, MotorType.kBrushless);
  private CANSparkMax l_belt = new CANSparkMax(CAN.MAG_L_BELT, MotorType.kBrushless);
  //private CANSparkMax v_belt = new CANSparkMax(CAN.MAG_v_belt, MotorType.kBrushless);
  
  /*Definitions*/
  final DigitalInput lightGate1 = new DigitalInput(DigitalIO.MAGAZINE_GATE1);
  final DigitalInput lightGate2 = new DigitalInput(DigitalIO.MAGAZINE_GATE2);
  final DigitalInput lightGate3 = new DigitalInput(DigitalIO.MAGAZINE_GATE3);

  //Constructor
  public Magazine_Subsystem() {
    // copy the PID settings to the hardware
    top_wheel = new TalonSRX(CAN.MAG_TOP_WHEEL);
    MagazineSettings.r_beltPIDF.copyTo(r_belt.getPIDController(), slot);
    MagazineSettings.l_beltPIDF.copyTo(l_belt.getPIDController(), slot);

    r_belt.clearFaults();
    r_belt.restoreFactoryDefaults();

    l_belt.clearFaults();
    l_belt.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //sets the belts to a speed
  public void driveWheelOn(double speed){
    top_wheel.set(TalonSRXControlMode.PercentOutput, speed);
    r_belt.set(speed);
    l_belt.set(-speed);
    //v_belt.set(speed);
  }

  public void driveWheelOff(){
    top_wheel.set(TalonSRXControlMode.PercentOutput, 0);
    r_belt.set(0);
    l_belt.set(0);
    //v_belt.set(speed);
  }

  //sets the roller wheel to a speed
  public void rollerWheelOn(double speed){
    r_belt.set(speed);
    l_belt.set(-speed);
  }

  public void rollerWheelOff(){
    r_belt.set(0);
    l_belt.set(0);
  } 

  //reverses direction of rotation to expell cargo
  public void expellCargo(double speed){
    top_wheel.set(TalonSRXControlMode.PercentOutput, -speed);
    r_belt.set(-speed);
    l_belt.set(speed);
        //v_belt.set(-speed);
  }
  
  //lets us know if cargo is detected
  public boolean isGate1Blocked() {
    return lightGate1.get();
  }
  public boolean isGate2Blocked(){
    return lightGate2.get();
  }
  public boolean isGate3Blocked(){
    return lightGate3.get();
  }
  //TODO Think about the API to expose for writing commands
  // TODO - Postion or velocity control?

}
