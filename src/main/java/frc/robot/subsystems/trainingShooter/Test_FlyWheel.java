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
}
