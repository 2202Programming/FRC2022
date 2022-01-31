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
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

public class DriveCmd extends CommandBase {

  public enum DriveModeTypes {
    robotCentric("Robot Centric"),
    fieldCentric("Field Centric"),
    hubCentric("Hub Centric");

    private String name;

    private DriveModeTypes(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
    }
  }

  final SwerveDrivetrain drivetrain;
  final DriverControls dc;
  final SwerveDriveKinematics kinematics;
  // command behaviors
  DriveModeTypes driveMode = DriveModeTypes.robotCentric;
  boolean fieldRelativeMode = false;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;

  // PID for heading to a target
  private PIDController anglePid;
  private double angle_kp = 0.07;
  private double angle_ki = 0.0;
  private double angle_kd = 0.0;

  // private Pose2d centerField = new Pose2d(27, 13.5, new Rotation2d()); //actual
  // hub location?
  private Pose2d centerField = new Pose2d(10, 0, new Rotation2d()); // close point for testing to max rotation obvious

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  private NetworkTableEntry driveCmd;
  private NetworkTableEntry NTDriveMode;
  private NetworkTableEntry fieldMode;
  private NetworkTableEntry hubCentricTarget;
  private NetworkTableEntry xVelTarget;
  private NetworkTableEntry yVelTarget;
  private NetworkTableEntry rotVelTarget;
  private NetworkTableEntry NTangleError;
  private NetworkTableEntry xJoystick;
  private NetworkTableEntry yJoystick;

  public final String NT_Name = "DT"; // expose data under DriveTrain table

  public DriveCmd(SwerveDrivetrain drivetrain, DriverControls dc) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
    this.kinematics = drivetrain.getKinematics();

    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    hubCentricTarget = table.getEntry("/hubCentricTarget");
    NTDriveMode = table.getEntry("/DriveMode");
    fieldMode = table.getEntry("/FieldMode");

    xVelTarget = table.getEntry("/xVelTarget");
    yVelTarget = table.getEntry("/yVelTarget");
    rotVelTarget = table.getEntry("/rotVelTarget");
    NTangleError = table.getEntry("/angleError");
    xJoystick = table.getEntry("/xJoystick");
    yJoystick = table.getEntry("/yJoystick");
    driveCmd = table.getEntry("/Drive Command");
  }

  public DriveCmd(SwerveDrivetrain drivetrain, DriverControls dc, boolean fieldRelativeMode) {
    this(drivetrain, dc);
    this.fieldRelativeMode = fieldRelativeMode;
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

    // Now workout drive mode behavior
    switch (driveMode) {
      case robotCentric:
        output_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        break;

      case fieldCentric:
        output_states = kinematics
            .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
        break;

      case hubCentric:
        rot = 0;

        // set goal of angle PID to be heading (in degrees) from current position to
        // centerfield
        double targetAngle = getHeading2Target(drivetrain.getPose(), centerField);
        double currentAngle = drivetrain.getPose().getRotation().getDegrees(); // from -180 to 180
        double angleError = targetAngle - currentAngle;
        // feed both PIDs even if not being used.
        anglePid.setSetpoint(targetAngle);
        double anglePidOutput = anglePid.calculate(currentAngle);
        rot = rotLimiter.calculate(anglePidOutput); // use Odometery PID for rotation in hub centric

        // deal with continuity issue across 0
        if (angleError < -180) {
          targetAngle += 360;
        }
        if (angleError > 180) {
          targetAngle -= 360;
        }
        hubCentricTarget.setValue(targetAngle);
        NTangleError.setDouble(angleError);

        output_states = kinematics
            .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
        break;
    }
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

  public void toggleFieldRealitiveMode() {
    if (fieldRelativeMode)
      fieldRelativeMode = false;
    else
      fieldRelativeMode = true;
    fieldMode.setBoolean(fieldRelativeMode);
    return;
  }

  public void cycleDriveMode() {
    switch (driveMode) {
      case robotCentric:
        driveMode = DriveModeTypes.fieldCentric;
        break;
      case fieldCentric:
        driveMode = DriveModeTypes.hubCentric;
        break;
      case hubCentric:
        driveMode = DriveModeTypes.robotCentric;
        break;
    }
    NTDriveMode.setString(driveMode.toString());
  }

  public DriveModeTypes getDriveMode() {
    return driveMode;
  }

  void updateNT() {
    // update network tables
    xJoystick.setDouble(dc.getVelocityX());
    yJoystick.setDouble(dc.getVelocityY());
    xVelTarget.setValue(xSpeed);
    yVelTarget.setValue(ySpeed);
    rotVelTarget.setValue(rot);
    fieldMode.setBoolean(fieldRelativeMode);
    NTDriveMode.setString(driveMode.toString());
    driveCmd.setString("DriveCmd");
  }

}
