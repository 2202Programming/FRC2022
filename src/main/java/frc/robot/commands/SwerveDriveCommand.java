package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final DriverControls dc;
  private PIDController anglePid;
  private double angle_kp = 0.04;
  private double angle_ki = 0.0;
  private double angle_kd = 0.0;
  //private Pose2d centerField = new Pose2d(27, 13.5, new Rotation2d()); //actual hub location?
  private Pose2d centerField = new Pose2d(5,5, new Rotation2d()); //close point for testing to max rotation obvious

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private NetworkTable table;
  private NetworkTableEntry hubCentricTarget;
  private NetworkTableEntry xVelTarget;
  private NetworkTableEntry yVelTarget;
  private NetworkTableEntry rotVelTarget;
  private NetworkTableEntry NTangleError;
  public final String NT_Name = "DT"; // expose data under DriveTrain table

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, DriverControls dc) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    hubCentricTarget = table.getEntry("/hubCentricTarget");
    xVelTarget = table.getEntry("/xVelTarget");
    yVelTarget = table.getEntry("/yVelTarget");
    rotVelTarget = table.getEntry("/rotVelTarget");
    NTangleError = table.getEntry("/angleError");
  }

  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = 0;

        //set goal of angle PID to be heading (in degrees) from current position to centerfield
        double targetAngle = getHeading(drivetrain.getPose(), centerField);
        double currentAngle = drivetrain.getPose().getRotation().getDegrees(); //from -180 to 180
        double angleError = targetAngle - currentAngle;
        //deal with continuity issue across 0
        if(angleError < -180) {
          targetAngle += 360;
        }
        if(angleError > 180) {
          targetAngle -= 360;
        }
        hubCentricTarget.setValue(targetAngle);
        NTangleError.setDouble(angleError);

    switch(drivetrain.getDriveMode()){
      case robotCentric:
        rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed; //use joystick for rotation in robot and field centric modes
        break;
      case fieldCentric:
        rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed; //use joystick for rotation in robot and field centric modes
        break;
      case hubCentric:
        anglePid.setSetpoint(targetAngle);
        rot = rotLimiter.calculate(anglePid.calculate(currentAngle)); //use PID for rotation in hub centric
        break;
    }
   
    drivetrain.drive(xSpeed, ySpeed, rot); 
    xVelTarget.setValue(xSpeed);
    yVelTarget.setValue(ySpeed);
    rotVelTarget.setValue(rot);
  }

  //takes 2 positions, gives heading from point A to point B (in degrees)
  private double getHeading(Pose2d a, Pose2d b) {

    //from -PI to +PI
    double theta = Math.atan2(b.getY()-a.getY(), b.getX()-a.getX());
    
    //convert this to degrees in the range -180 to 180
    theta = Math.toDegrees(theta);

    return theta;
  }

}
