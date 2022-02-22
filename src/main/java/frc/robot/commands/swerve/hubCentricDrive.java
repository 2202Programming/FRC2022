package frc.robot.commands.swerve;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

/* Current driving behavior:
  Starts in field centric
  B will toggle between field centric and intake centric
  Holding right trigger will switch to hub centric until you let go, then it will go back to original mode
          (either field or intake centric, depending what you started in)
  If in intake centric and you try to rotate with left joystick, will drop back to field centric mode.
*/


public class hubCentricDrive extends CommandBase {

  final SwerveDrivetrain drivetrain;
  final DriverControls dc;
  final SwerveDriveKinematics kinematics;
  final Limelight_Subsystem limelight;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;

  // PID for odometery-based heading to a target
  private PIDController anglePid;
  private double angle_kp = 0.075;
  private double angle_ki = 0.004;
  private double angle_kd = 0.005;

  // PID for limelight-based heading to a target
  PIDController limelightPid;
  double limelight_kP = 0.05;
  double limelight_kI = 0.0;
  double limelight_kD = 0.0;
  double limelightPidOutput = 0.0;
  
  private Pose2d centerField = new Pose2d(27, 13.5, new Rotation2d()); //actual
  // hub location?
  //private Pose2d centerField = new Pose2d(10, 0, new Rotation2d()); // close point for testing to max rotation obvious

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter llLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  private NetworkTableEntry hubCentricTarget;
  private NetworkTableEntry NTangleError;
  public final String NT_Name = "DC"; // expose data under Drive Controller table

  double log_counter = 0;
  double currentAngle;
  double angleError;
  double targetAngle;

  public hubCentricDrive(SwerveDrivetrain drivetrain, DriverControls dc, Limelight_Subsystem limelight) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
    this.kinematics = drivetrain.getKinematics();
    this.limelight = limelight;

    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    limelightPid = new PIDController(limelight_kP, limelight_kI, limelight_kD);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    hubCentricTarget = table.getEntry("/hubCentricTarget");
    NTangleError = table.getEntry("/angleError");

  }

  @Override
  public void initialize() {
    updateNT();
  }

  void calculate() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;
    rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    rot = MathUtil.clamp(rot, -Constants.DriveTrain.kMaxAngularSpeed, Constants.DriveTrain.kMaxAngularSpeed);

    currrentHeading = drivetrain.getPose().getRotation();

    rot = 0;
    // set goal of angle PID to be heading (in degrees) from current position to
    // centerfield
    targetAngle = getHeading2Target(drivetrain.getPose(), centerField);
    targetAngle = targetAngle + 180; // flip since shooter is on "back" of robot
    if(targetAngle > 180){
      targetAngle = targetAngle - 360;
    }
    currentAngle = drivetrain.getPose().getRotation().getDegrees(); // from -180 to 180
    angleError = targetAngle - currentAngle;
    anglePid.setSetpoint(targetAngle);
    rot = anglePid.calculate(currentAngle);

    if (limelight.getTarget() && limelight.getLEDStatus()) {
      // if limelight is available, override rotation input from odometery to limelight
      limelightPid.setSetpoint(0); // always go towards the light.
      limelightPidOutput = limelightPid.calculate(limelight.getFilteredX());
      // update rotation and calulate new output-states
      rot = llLimiter.calculate(limelightPidOutput);
    }
    
    // deal with continuity issue across 0 //not sure this is necessary, it's happening after PID calculations...
    if (angleError < -180) {
      targetAngle += 360;
    }
    if (angleError > 180) {
      targetAngle -= 360;
    }
    output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));

    }

  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
    updateNT();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // takes 2 positions, gives heading from current point to target (in degrees)
  double getHeading2Target(Pose2d current, Pose2d target) {
    // from -PI to +PI
    double theta = Math.atan2(target.getY() - current.getY(), target.getX() - current.getX());

    // convert this to degrees in the range -180 to 180
    theta = Math.toDegrees(theta);
    return theta;
  }


  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
    // update network tables
    hubCentricTarget.setValue(targetAngle);
    NTangleError.setDouble(angleError);
    }
  }
}
