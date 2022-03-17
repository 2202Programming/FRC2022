package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.util.PoseMath;

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
  private double angle_kp = 4.0;
  private double angle_ki = 0;
  private double angle_kd = 0.001;

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
  public final String NT_Name = "Shooter"; 

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

    // anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    anglePid.enableContinuousInput(-Math.PI, Math.PI);
    limelightPid = new PIDController(limelight_kP, limelight_kI, limelight_kD);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    hubCentricTarget = table.getEntry("/HubCentric/hubCentricTarget");
    NTangleError = table.getEntry("/HubCentric/angleError");
    NTvelocityCorrectionAngle = table.getEntry("/HubCentric/VelCorrectionAngle");

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

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);

    // set goal of angle PID to be heading (in rad) from current position to
    // centerfield
    targetAngle = PoseMath.getHeading2Target(drivetrain.getPose(), Autonomous.hubPose);
    targetAngle = targetAngle.plus(new Rotation2d(Math.PI)); // flip since shooter is on "back" of robot, bound to -pi to +pi
    currentAngle = drivetrain.getPose().getRotation(); // from -pi to pi

    //get correction angle for velocity based on our velocity vector to hub
    // velocityCorrectionAngle = PoseMath.angleVirtualTarget(drivetrain.getPose(), Autonomous.hubPose, adjustHubPosition());

    //uncomment this when ready to test velocity correction
    //targetAngle.minus(velocityCorrectionAngle); //might need to be plus, depending on direction of travel??
    //Also, this correction angle is probably for the intake side ("front of robot") and may need a PI inversion so the shooter is pointing to hub

    anglePid.setSetpoint(targetAngle.getRadians()); 
    rot = anglePid.calculate(currentAngle.getRadians());

    //angleError is just for reporting
    angleError = targetAngle;
    angleError = angleError.minus(currentAngle);

    if (limelight.getTarget() && limelight.getLEDStatus()) {
      // if limelight is available, override rotation input from odometery to limelight
      // limelight is on the shooter side, so we don't need to worry about flipping target angles
      limelightPid.setSetpoint(0);
      //uncomment this below and comment line above when ready to test velocity correction
      //limelightPid.setSetpoint(velocityCorrectionAngle.getDegrees()*Shooter.degPerPixel); // 0 is towards target, 
      //adjust based on velocity
      limelightPidOutput = limelightPid.calculate(limelight.getFilteredX());
      angleError = Rotation2d.fromDegrees(limelight.getFilteredX()*Shooter.degPerPixel); //approximation of degrees off center
      // update rotation and calulate new output-states
      rot = llLimiter.calculate(limelightPidOutput);
    }

    output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentAngle));
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

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
    // update network tables
    hubCentricTarget.setDouble(targetAngle.getDegrees());
    NTangleError.setDouble(angleError.getDegrees());
    // NTvelocityCorrectionAngle.setDouble(velocityCorrectionAngle.getDegrees());
    }
  }

  public Rotation2d getAngleError() {
    return this.angleError;
  }

  //returns a position for the hub adjusted for robot movement
  public Pose2d adjustHubPosition(){
    final double HANGTIME = 1.5; //needs to be measured, probably a trendline equation
    double[] u = {drivetrain.getChassisSpeeds().vxMetersPerSecond, drivetrain.getChassisSpeeds().vxMetersPerSecond}; //robot's direction vector
    double robotX = drivetrain.getPose().getX(); //x position of robot
    double robotY = drivetrain.getPose().getY(); //y position of robot
    double hubX = Constants.Autonomous.hubPose.getX(); //real hub x
    double hubY = Constants.Autonomous.hubPose.getY(); //real hub y
    double[] v = {hubY-robotY, robotX-hubX}; //tangent vector to hub
    Rotation2d oneAndTwo = Rotation2d.fromDegrees(drivetrain.getBearing()).plus(PoseMath.getHeading2Target(drivetrain.getPose(), Constants.Autonomous.hubPose));
    Rotation2d angle3 = new Rotation2d(90).minus(oneAndTwo); //angle in degrees

    /*find projection of u onto v*/
    double projectionMagnitude = (u[0]*v[0]+u[1]*v[1]) / (Math.pow(v[0], 2) + Math.pow(v[1],2));
    double[] tangentVector = {projectionMagnitude*v[0], projectionMagnitude*v[1]};

    double offsetDistance = Math.sqrt(Math.pow(tangentVector[0], 2) + Math.pow(tangentVector[1], 2)) * HANGTIME;

    Rotation2d temp = angle3.plus(new Rotation2d(180));
    double xOffset = Math.cos(temp.getDegrees())*offsetDistance; 
    double yOffset = Math.sin(temp.getDegrees())*offsetDistance;

    double newX = xOffset + hubX;
    double newY = yOffset + hubY;

    return new Pose2d(newX, newY, new Rotation2d());
  }

}
