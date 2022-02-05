// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CAN;
import static frc.robot.Constants.MagazineSettings;
import static frc.robot.Constants.DigitalIO;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine_Subsystem extends SubsystemBase {
  //slot to use on controllers
  int slot = 0;
  
  /** Creates a new Magazine2. */
  private TalonSRX h_belt;
  //private CANSparkMax v_belt = new CANSparkMax(CAN.MAG_v_belt, MotorType.kBrushless);
  
  /*Definitions*/
  final DigitalInput lightGate = new DigitalInput(DigitalIO.MAGAZINE_GATE);


  //Constructor
  public Magazine_Subsystem() {
    // copy the PID settings to the hardware
    h_belt = new TalonSRX(Constants.CAN.MAG_h_belt);
    //MagazineSettings.h_beltPIDF.copyTo(h_belt.getPIDController(), slot);
    //MagazineSettings.v_beltPIDF.copyTo(v_belt.getPIDController(), slot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //sets the belts to a speed
  public void beltOn(double speed){
    h_belt.set(TalonSRXControlMode.PercentOutput, speed);
    //v_belt.set(speed);
  }
  //turns belts off
  public void beltOff(){
    h_belt.set(TalonSRXControlMode.PercentOutput, 0);
    //v_belt.set(0);
  }
  //reverses direction of rotation to expell cargo
  public void expellCargo(double speed){
   //h_belt.set(-speed);
    //v_belt.set(-speed);
  }
  
  //lets us know if cargo is detected
  public boolean isGateBlocked() {
    return lightGate.get();
  }

  
//snowblower motor

  //TODO Think about the API to expose for writing commands
  // TODO - Postion or velocity control?

}
