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
    hubCentric("Hub Centric"),
    shootingRecovery("Shooting Recovery"),
    shootingCentric("Shooting Centric"),
    intakeCentric("Intake Centric");

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
  DriveModeTypes lastDriveMode = DriveModeTypes.robotCentric;
  boolean fieldRelativeMode = false;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;

  // PID for heading to a target
  private PIDController anglePid;
  private double angle_kp = 0.075;
  private double angle_ki = 0.004;
  private double angle_kd = 0.005;

  // private Pose2d centerField = new Pose2d(27, 13.5, new Rotation2d()); //actual
  // hub location?
  private Pose2d centerField = new Pose2d(10, 0, new Rotation2d()); // close point for testing to max rotation obvious

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  private NetworkTableEntry driveCmd;
  private NetworkTableEntry fieldMode;
  private NetworkTableEntry hubCentricTarget;
  private NetworkTableEntry xVelTarget;
  private NetworkTableEntry yVelTarget;
  private NetworkTableEntry rotVelTarget;
  private NetworkTableEntry NTangleError;
  private NetworkTableEntry xJoystick;
  private NetworkTableEntry yJoystick;
  public final String NT_Name = "DT"; // expose data under DriveTrain table

  double log_counter = 0;

  public DriveCmd(SwerveDrivetrain drivetrain, DriverControls dc2) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc2;
    this.kinematics = drivetrain.getKinematics();

    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    hubCentricTarget = table.getEntry("/hubCentricTarget");;
    fieldMode = table.getEntry("/FieldMode");

    xVelTarget = table.getEntry("/xVelTarget");
    yVelTarget = table.getEntry("/yVelTarget");
    rotVelTarget = table.getEntry("/rotVelTarget");
    NTangleError = table.getEntry("/angleError");
    xJoystick = table.getEntry("/xJoystick");
    yJoystick = table.getEntry("/yJoystick");
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
        drivetrain.setDriveModeString("Robot Centric Drive");
        output_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        break;

      case fieldCentric:
        drivetrain.setDriveModeString("Field Centric Drive");
        output_states = kinematics
            .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
        break;

      case shootingCentric: //same behavior as hubCentric for now
      case hubCentric:
        rot = 0;
        drivetrain.setDriveModeString("Hub Centric Drive");
        // set goal of angle PID to be heading (in degrees) from current position to
        // centerfield
         double targetAngle = getHeading2Target(drivetrain.getPose(), centerField);
        double currentAngle = drivetrain.getPose().getRotation().getDegrees(); // from -180 to 180
        double angleError = targetAngle - currentAngle;
        // feed both PIDs even if not being used.
        anglePid.setSetpoint(targetAngle);
        rot = anglePid.calculate(currentAngle);
        
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

      case shootingRecovery:
        drivetrain.setDriveModeString("Shooting Recovery");
        // set goal of angle PID to be heading (in degrees) current bearing
        double m_targetAngle = drivetrain.getBearing();
        double m_currentAngle = drivetrain.getPose().getRotation().getDegrees(); // from -180 to 180
        double m_angleError = m_targetAngle - m_currentAngle;
        // feed both PIDs even if not being used.
        anglePid.setSetpoint(m_targetAngle);
        rot = anglePid.calculate(m_currentAngle);
        
        // deal with continuity issue across 0
        if (m_angleError < -180) {
          m_targetAngle += 360;
        }
        if (m_angleError > 180) {
          m_targetAngle -= 360;
        }
        hubCentricTarget.setValue(m_targetAngle);
        NTangleError.setDouble(m_angleError);

        //heading is close enough to bearing, release rotational control back to drivers
        if(m_angleError < 1){
          driveMode = lastDriveMode;
        }

        output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
        break;
      case intakeCentric:
        // set goal of angle PID to be heading (in degrees) current bearing
        double m_targetAngle2 = drivetrain.getBearing();
        double m_currentAngle2 = drivetrain.getPose().getRotation().getDegrees(); // from -180 to 180
        double m_angleError2 = m_targetAngle2 - m_currentAngle2;
        // feed both PIDs even if not being used.
        anglePid.setSetpoint(m_targetAngle2);
        rot = anglePid.calculate(m_currentAngle2);
        
        // deal with continuity issue across 0
        if (m_angleError2 < -180) {
          m_targetAngle2 += 360;
        }
        if (m_angleError2 > 180) {
          m_targetAngle2 -= 360;
        }
        hubCentricTarget.setValue(m_targetAngle2);
        NTangleError.setDouble(m_angleError2);

        output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));

      break;
    }
  }

  @Override
  public void execute() {
    
    //trigger is being pressed, drive in shooting mode
    if(drivetrain.getShootingMode()){
      driveMode = DriveModeTypes.shootingCentric;
    }

    //this means trigger just got released, time to start recovery mode
    if(!(drivetrain.getShootingMode()) && (driveMode == DriveModeTypes.shootingCentric)){ 
      driveMode = DriveModeTypes.shootingRecovery;
    }
    
    calculate();
    drivetrain.drive(output_states);
    updateNT();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setDriveModeString("None");
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
    lastDriveMode = driveMode;
    switch (driveMode) {
      case robotCentric:
        driveMode = DriveModeTypes.fieldCentric;
        drivetrain.setDriveModeString("Robot Centric Drive");
        break;
      case fieldCentric:
        driveMode = DriveModeTypes.hubCentric;
        drivetrain.setDriveModeString("Field Centric Drive");
        break;
      case hubCentric:
        driveMode = DriveModeTypes.intakeCentric;
        drivetrain.setDriveModeString("Hub Centric Drive");
        break;
      case intakeCentric:
        driveMode = DriveModeTypes.robotCentric;
        drivetrain.setDriveModeString("Intake Centric Drive");
        break;
    }
  }

  public DriveModeTypes getDriveMode() {
    return driveMode;
  }

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
    // update network tables
    xJoystick.setDouble(dc.getVelocityX());
    yJoystick.setDouble(dc.getVelocityY());
    xVelTarget.setValue(xSpeed);
    yVelTarget.setValue(ySpeed);
    rotVelTarget.setValue(rot);
    fieldMode.setBoolean(fieldRelativeMode);
    driveCmd.setString("DriveCmd");
    }
  }

}
