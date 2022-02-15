package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

public class Ultrasonic extends TimedRobot {

  public DigitalOutput ultrasonicTriggerPinOne = new DigitalOutput(0);
  
  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public double ultrasonicSensorOneRange = 0;

  public double voltageScaleFactor = 1;

  
  public void turnOnSensorOne() {
    ultrasonicTriggerPinOne.set(true);
  }

  public void turnOffSensorOne() {
    ultrasonicTriggerPinOne.set(false);
  }

  @Override
  public void robotInit() {
    //Initialize range readings on SmartDashboard as max distance in Centimeters.
    SmartDashboard.putNumber("Sensor 1 Range", 500);

  }
  @Override
  public void robotPeriodic() {
    //Publish range readings to SmartDashboard
    SmartDashboard.putNumber("Sensor 1 Range", ultrasonicSensorOneRange);

    voltageScaleFactor = 5 / RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at
  }
  @Override
  public void autonomousInit() {
    //If we are in Autonomous mode, turn on the first sensor (and turn off the second sensor)
    turnOnSensorOne();
  }
  @Override
  public void autonomousPeriodic() {
    //Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to Centimeters
    ultrasonicSensorOneRange = ultrasonicSensorOne.getValue()*voltageScaleFactor*0.125;

  }
  @Override
  public void teleopInit() {
    turnOnSensorOne();
  }
  

  @Override
  public void teleopPeriodic() {
    //Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to Centimeters
    ultrasonicSensorOneRange = ultrasonicSensorOne.getValue()*voltageScaleFactor*0.125;
  }

  @Override
  public void disabledInit() {
    //If the robot is disabled, turn off both sensors
    turnOffSensorOne();
  }

}

