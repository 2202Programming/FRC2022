package frc.robot.subsystems.trainingShooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Test_FlyWheel {
    public static class FlyWheelConfig {
        public PIDFController pid;
        public double maxOpenLoopRPM;
        public double gearRatio; // account for gearbox reduction to flywheel
        public boolean sensorPhase;
        public boolean inverted;
        public double flywheelRadius;
        public double FWrpe2MU; // FlywheelRPM to Motor-Units (includes gearing)
      };


      // Talon Slot stuff, we just use slot 0
  final int kPIDLoopIdx = 0;
  final int kTimeoutMs = 30;

  TalonSRXConfiguration srxconfig;
  WPI_TalonSRX motor; // this could be a generic motor controller...

  final double FWrpm2Counts; // flywheel RPM given motor-unit counts (f(gear, meas-period))
  final double MUCounts2FWrpm; // motor units (counts/100ms) to FW RPM (1/FWrpm2Counts)

  public FlyWheelConfig cfg;

  Test_FlyWheel(int CAN_ID, FlyWheelConfig cfg) {
    //TALONSRXCONFIGURATION
    //WPI_TALONSRX
    //FLYWHEELCONFIGURATION
    srxconfig = new TalonSRXConfiguration(); 
    motor = new WPI_TalonSRX(CAN_ID);
    motor.setInverted(cfg.inverted);
    this.cfg = cfg;

    // flywheel constants RPM given motor-unit counts (f(gear, meas-period))
     // motor counts are bigger, motor spins faster than FW
  // motor units (counts/100ms) to FW RPM

    // use max rpm and max motor out to calculate kff
    
    //Laster stuff

  }

  //Write a function to get RPM
  public double getRPM(){
    return motor.getSelectedSensorVelocity();
  }
  //Write a function to get motor output as a percent 
  public double getPercent(){
    return motor.getMotorOutputPercent();

  }

  //write a function to set the RPM
  public void setRPM(double RPM ){
    motor.set(ControlMode.Velocity,RPM);

  }
  //write a function to motor to percent
  public void setMotorToPercent(double percentage) {
    motor.set(ControlMode.PercentOutput,percentage);
  }
  //write a function to set PID
  public void setPID(double KP, double KI, double KD) {
    cfg.pid.setPID(KP,KI,KD);
  }
  //write a function to get P
  public double getKP() {
    return cfg.pid.getP();
  }
  //write a function to get I
  public double getKI() {
    return cfg.pid.getI();
  }

  //write a funciton to get D
  public double getKD() {
    return cfg.pid.getD();
  }
}
