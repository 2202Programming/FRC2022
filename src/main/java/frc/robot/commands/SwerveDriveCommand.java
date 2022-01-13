package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final DriverControls dc;
  private PIDController anglePid;
  private double angle_kp = 1.0;
  private double angle_ki = 0.0;
  private double angle_kd = 0.0;
  private Pose2d centerField = new Pose2d(27, 13.5, new Rotation2d());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, DriverControls dc) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
  }

  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
      //-xspeedLimiter.calculate(controller.getY(GenericHID.Hand.kLeft))
      //  * DriveTrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;
    //  -yspeedLimiter.calculate(controller.getX(GenericHID.Hand.kLeft))
    //    * DriveTrain.kMaxSpeed;

    //set goal of angle PID to be heading (in degrees) from current position to centerfield
    anglePid.setSetpoint(getHeading(drivetrain.getPose(), centerField));


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot;
    if (!(drivetrain.getDriveMode() == 2)) {
      rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;
    } else {
      rot = rotLimiter.calculate(anglePid.calculate(drivetrain.getPose().getRotation().getDegrees())) * DriveTrain.kMaxAngularSpeed;
    }
      //-rotLimiter.calculate(controller.getX(GenericHID.Hand.kRight))
      //  * DriveTrain.kMaxAngularSpeed;
    
    drivetrain.drive(xSpeed, ySpeed, rot); //for testing, bring up rot first
  }

  //takes 2 positions, gives heading from point A to point B (in degrees)
  private double getHeading(Pose2d a, Pose2d b) {
    double theta = Math.atan2(b.getX()-a.getX(), b.getY()-a.getY());
    if(theta < 0){
        theta += 2*Math.PI;
    }
    return theta;
  }

}
