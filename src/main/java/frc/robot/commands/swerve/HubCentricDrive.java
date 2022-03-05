package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.Shooter;
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


public class HubCentricDrive extends CommandBase {

  final SwerveDrivetrain drivetrain;
  final DriverControls dc;
  final SwerveDriveKinematics kinematics;
  final Limelight_Subsystem limelight;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
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
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter llLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  private NetworkTableEntry hubCentricTarget;
  private NetworkTableEntry NTangleError;
  private NetworkTableEntry NTvelocityCorrectionAngle;
  public final String NT_Name = "DC"; // expose data under Drive Controller table

  double log_counter = 0;
  Rotation2d currentAngle;
  Rotation2d angleError;
  Rotation2d targetAngle;
  Rotation2d velocityCorrectionAngle;

  public HubCentricDrive(SwerveDrivetrain drivetrain, DriverControls dc, Limelight_Subsystem limelight) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
    this.kinematics = drivetrain.getKinematics();
    this.limelight = limelight;

    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    anglePid.enableContinuousInput(-Math.PI, Math.PI);
    limelightPid = new PIDController(limelight_kP, limelight_kI, limelight_kD);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    hubCentricTarget = table.getEntry("/hubCentricTarget");
    NTangleError = table.getEntry("/angleError");
    NTvelocityCorrectionAngle = table.getEntry("/VelCorrectionAngle");

    calculate();

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

    // set goal of angle PID to be heading (in rad) from current position to
    // centerfield
    targetAngle = getHeading2Target(drivetrain.getPose(), Autonomous.hubPose);
    targetAngle.plus(new Rotation2d(Math.PI)); // flip since shooter is on "back" of robot, bound to -pi to +pi
    currentAngle = drivetrain.getPose().getRotation(); // from -pi to pi
    angleError = targetAngle;
    angleError.minus(currentAngle);
    velocityCorrectionAngle = velCorrectOdometerAngle();
    targetAngle.minus(velocityCorrectionAngle); //might need to be plus, depending on direction of travel??
    anglePid.setSetpoint(targetAngle.getDegrees()); //PID was tuned in degrees already
    rot = anglePid.calculate(currentAngle.getDegrees());

    if (limelight.getTarget() && limelight.getLEDStatus()) {
      // if limelight is available, override rotation input from odometery to limelight
      limelightPid.setSetpoint(velocityCorrectionAngle.getDegrees()*Shooter.degPerPixel); // 0 is towards target, adjust based on velocity
      limelightPidOutput = limelightPid.calculate(limelight.getFilteredX());
      angleError = Rotation2d.fromDegrees(limelight.getFilteredX()*Shooter.degPerPixel); //approximation of degrees off center
      // update rotation and calulate new output-states
      rot = llLimiter.calculate(limelightPidOutput);
    }

    output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentAngle));
  }

  //return Rot2d of correction to original target angle based on velocity and hangtime
  private Rotation2d velCorrectOdometerAngle(){
    //distance to target is PoseMath.poseDistance(RobotContainer.RC().drivetrain.getPose(), Autonomous.hubPose);
    //drivetrain has getBearing and getVelocity methods
    //local method getHeading2Target(drivetrain.getPose(), Autonomous.hubPose);
    //estimate hangtime a 1.5s for now, later will look up based on distance
    return new Rotation2d(0); //do something fancier based on robot motion
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
  Rotation2d getHeading2Target(Pose2d current, Pose2d target) {
    // from -PI to +PI
    return new Rotation2d(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
  }


  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
    // update network tables
    hubCentricTarget.setDouble(targetAngle.getDegrees());
    NTangleError.setDouble(angleError.getDegrees());
    NTvelocityCorrectionAngle.setDouble(velocityCorrectionAngle.getDegrees());
    }
  }

  public Rotation2d getAngleError() {
    return this.angleError;
  }

}
