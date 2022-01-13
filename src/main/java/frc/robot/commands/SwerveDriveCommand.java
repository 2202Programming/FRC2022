package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final DriverControls dc;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, DriverControls dc) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
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

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;
      //-rotLimiter.calculate(controller.getX(GenericHID.Hand.kRight))
      //  * DriveTrain.kMaxAngularSpeed;
    
    drivetrain.drive(xSpeed, ySpeed, rot); //for testing, bring up rot first
  }

}
