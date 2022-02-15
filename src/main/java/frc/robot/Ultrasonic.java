package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

// Creates a ping-response Ultrasonic object on DIO 1 and 2.
Ultrasonic ultrasonic = new Ultrasonic(1, 2);

// Initialize motor controllers and drive
Spark left1 new Spark(0);
Spark left2 = new Spark(1);

Spark right1 = new Spark(2);
Spark right2 = new Spark(3);

MotorControllerGroup leftMotors = new MotorControllerGroup(left1, left2);
MotorControllerGroup rightMotors = new MotorControllerGroup(right1, right2);

DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

@Override
public void robotInit() {
    // Start the ultrasonic in automatic mode
    Ultrasonic.setAutomaticMode(true);
}

@Override
public void autonomousPeriodic() {
    if(ultrasonic.GetRangeInches() > 12) {
        drive.tankDrive(.5, .5);
    }
    else {
        drive.tankDrive(0, 0);
    }
}

Ultrasonic ultrasonic = new Ultrasonic(1, 2);

public void robotInit() {
    // Places a the ultrasonic on the dashboard
    Shuffleboard.getTab("Example tab").add(ultrasonic);
