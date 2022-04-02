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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

  // PID for limelight-based heading to a target
  PIDController limelightPid;
  double limelight_kP = Shooter.limelight_default_p;
  double limelight_kI = Shooter.limelight_default_i;
  double limelight_kD = Shooter.limelight_default_d;
  double limelightPidOutput = 0.0;
  
  double r_limelight_kP = limelight_kP;
  double r_limelight_kI = limelight_kI;
  double r_limelight_kD = limelight_kD;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter llLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  private NetworkTableEntry NTangleError;
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
    limelightPid = new PIDController(limelight_kP, limelight_kI, limelight_kD);
    limelightPid.setTolerance(Shooter.angleErrorTolerance, Shooter.angleVelErrorTolerance);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    NTangleError = table.getEntry("/HubCentric/angleError");

    calculate();

  }

  @Override
  public void initialize() {
    updateNT();
  }

  void calculate() {
    final double min_rot_rate = 2.0; // [deg/s]
    final double max_rot_rate = 5.0;  //[deg/s]
    double llx = limelight.getX();  //[deg error]

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);

    // limelight is on the shooter side, so we don't need to worry about flipping target angles
    limelightPid.setSetpoint(0);

    //uncomment this below and comment line above when ready to test velocity correction
    //limelightPid.setSetpoint(velocityCorrectionAngle.getDegrees()*Shooter.degPerPixel); // 0 is towards target, 
   
    limelightPidOutput = limelightPid.calculate(llx);
    SmartDashboard.putNumber("LLPidOutput", limelightPidOutput);
    SmartDashboard.putNumber("LLFiltered", limelight.getFilteredX());
    angleError = Rotation2d.fromDegrees(limelight.getX()); //approximation of degrees off center

    // Clamp rotation rate to +/- X degrees/sec
    double min_rot = (Math.abs(llx) > 1.0)  ?  Math.signum(llx) *min_rot_rate : 0.0;
    rot = MathUtil.clamp(limelightPidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3;   //clamp in [deg/s] convert to [rad/s]
     
    // update rotation and calulate new output-states
    ///rot = llLimiter.calculate(limelightPidOutput) / 57.3; //degrees to radians/sec
   
    currentAngle = drivetrain.getPose().getRotation();
    output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentAngle));

    SmartDashboard.putNumber("Requested Limelight P", r_limelight_kP);
    SmartDashboard.putNumber("Requested Limelight I", r_limelight_kI);
    SmartDashboard.putNumber("Requested Limelight D", r_limelight_kD);
  }

  @Override
  public void execute() {
    pidPrint();
    pidSet();
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
      NTangleError.setDouble(angleError.getDegrees());
    }
  }

  public Rotation2d getAngleError() {
    return this.angleError;
  }

  public boolean isReady() {
    return limelightPid.atSetpoint();
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

  private void pidPrint(){
    SmartDashboard.putNumber("Current Limelight P", limelight_kP);
    SmartDashboard.putNumber("Current Limelight I", limelight_kI);
    SmartDashboard.putNumber("Current Limelight D", limelight_kD);

    // SmartDashboard.putNumber("Requested Limelight P", r_limelight_kP);
    // SmartDashboard.putNumber("Requested Limelight I", r_limelight_kI);
    // SmartDashboard.putNumber("Requested Limelight D", r_limelight_kD);
  }

  private void pidSet(){
    r_limelight_kP = SmartDashboard.getNumber("Requested Limelight P", r_limelight_kP);
    r_limelight_kI = SmartDashboard.getNumber("Requested Limelight I", r_limelight_kI);
    r_limelight_kD = SmartDashboard.getNumber("Requested Limelight D", r_limelight_kD);
    if((r_limelight_kP!=limelight_kP) || (r_limelight_kI!=limelight_kI) || (r_limelight_kD != limelight_kD)){
      limelight_kP=r_limelight_kP;
      limelight_kI=r_limelight_kI;
      limelight_kD=r_limelight_kD;
      limelightPid.setPID(limelight_kP, limelight_kI, limelight_kD);
    }

  }

}
