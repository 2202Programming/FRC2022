// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.Shooter;


/** Add your docs here. */
public class TalonFlywheel {
    //Talon Slot stuff, we just use slot 0
    final int kPIDLoopIdx = 0;
    final int kTimeoutMs = 30;

    TalonSRXConfiguration srxconfig;
    WPI_TalonSRX motor;     //this could be a generic motor controller...
    
    final double FWrpm2Counts;    // flywheel RPM given motor-unit counts (f(gear, meas-period))
    final double MUCounts2FWrpm;  // motor units (counts/100ms) to FW RPM (1/FWrpm2Counts)

    public TalonFlywheel(int CAN_ID, FlywheelConfig cfg) {
      srxconfig = new TalonSRXConfiguration();
      motor = new WPI_TalonSRX(CAN_ID);
      motor.setInverted(cfg.inverted);
    
      // flywheel constants RPM given motor-unit counts (f(gear, meas-period))
      FWrpm2Counts = Shooter.kRPM2Counts * cfg.gearRatio;  //motor counts are bigger, motor spins faster than FW    
      MUCounts2FWrpm  = 1.0 / FWrpm2Counts;  // motor units (counts/100ms) to FW RPM 

      // use max rpm and max motor out to calculate kff
      double kff = Shooter.kMaxMO / (cfg.maxOpenLoopRPM * FWrpm2Counts);
      cfg.pid.setF(kff);

      ErrorCode lasterr = motorConfig(cfg);
      if (lasterr.value != 0 ) {
        System.out.println("Flywheel motor error:" + lasterr.value + "  CANID=" + CAN_ID);
      }
    }

    ErrorCode motorConfig(FlywheelConfig cfg) {
      /* Factory Default all hardware to prevent unexpected behaviour */
      motor.configFactoryDefault();
      
      // use the config to set all values at once
      cfg.pid.copyTo(srxconfig.slot0);

      srxconfig.slot1 = srxconfig.slot0;
      motor.configAllSettings(srxconfig);

      /* Config sensor used for Primary PID [Velocity] */
      motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
      motor.setSensorPhase(cfg.sensorPhase);   // fix feedback direction
      motor.setNeutralMode(NeutralMode.Coast);

      /* Config the peak and nominal outputs */
      motor.configNominalOutputForward(0, kTimeoutMs);
      motor.configNominalOutputReverse(0, kTimeoutMs);
      motor.configPeakOutputForward(1, kTimeoutMs);
      motor.configPeakOutputReverse(-1, kTimeoutMs);
      return motor.getLastError();
    }

    /**
     * Gets RPM as measured at the flywheel 
     * @return flywheel_rpm
     */
    public double getRPM() {
      double vel_mu = motor.getSelectedSensorVelocity();   //motor units
      return vel_mu * MUCounts2FWrpm;   
    }
    public double getMotorOutputPercent() {
      return motor.getMotorOutputPercent();
    }

    public void  setRPM(double fw_rpm) {
      double sp = fw_rpm * FWrpm2Counts;
      motor.set(ControlMode.Velocity, sp);
    }

    public void setPercent(double pct) {
      motor.set(ControlMode.PercentOutput, pct); 
    }
  } //FlyWheel
