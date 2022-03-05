package frc.robot.subsystems.trainingShooter;

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

  FlyWheel(int CAN_ID, FlyWheelConfig cfg) {
    //TALONSRXCONFIGURATION
    //WPI_TALONSRX
    //FLYWHEELCONFIGURATION

    // flywheel constants RPM given motor-unit counts (f(gear, meas-period))
     // motor counts are bigger, motor spins faster than FW
  // motor units (counts/100ms) to FW RPM

    // use max rpm and max motor out to calculate kff
    
    //Laster stuff

  }

  //Write a function to get RPM

  //Write a function to get motor output as a percent 


  //write a function to set the RPM

  //write a function to motor to percent

  //write a function to set PID

  //write a function to get P

  //write a function to get I


  //write a funciton to get D

}
