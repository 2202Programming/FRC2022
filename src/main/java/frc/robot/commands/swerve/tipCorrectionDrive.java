package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

/* Current driving behavior:
  Starts in field centric
  B will toggle between field centric and intake centric
  Holding right trigger will switch to hub centric until you let go, then it will go back to original mode
          (either field or intake centric, depending what you started in)
  If in intake centric and you try to rotate with left joystick, will drop back to field centric mode.
*/

    // Drive Can 25 is spinning instead of Drive Can 22 on Roll towards negative 
public class tipCorrectionDrive extends FieldCentricDrive {

  double log_counter = 0;
    // Right is negative
  PIDController tipRollPid;
  double roll_kP = 1.0;
  double roll_kI = 0.0;
  double roll_kD = 0.0;
  double tipRollPidOutput = 0.0;
    // Back is negative
  PIDController tipPitchPid;
  double pitch_kP = 1.0;
  double pitch_kI = 0.0;
  double pitch_kD = 0.0;
  double tipPitchPidOutput = 0.0;

  NetworkTable table;
  NetworkTableEntry nt_roll_factor;
  NetworkTableEntry nt_pitch_factor;
  public final String NT_Name = "DC"; 

  public tipCorrectionDrive(SwerveDrivetrain drivetrain, DriverControls dc) {
    super(drivetrain, dc);
    addRequirements(drivetrain);
    tipRollPid = new PIDController(roll_kP, roll_kI, roll_kD);
    tipPitchPid = new PIDController(pitch_kP, pitch_kI, pitch_kD);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    nt_roll_factor = table.getEntry("/DriveController/RollFactor");
    nt_pitch_factor = table.getEntry("/DriveController/PitchFactor");
  }

  @Override
  public void initialize() {
    System.out.println("***STARTING TIP CORRECTION MODE");
  }

  void calculate() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    //xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    //ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;
    //rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;

    tipRollPid.setSetpoint(0); 
    tipRollPidOutput = tipRollPid.calculate(-RobotContainer.RC().sensors.getRoll());
    nt_roll_factor.setDouble(tipRollPidOutput);

    tipPitchPid.setSetpoint(0); 
    tipPitchPidOutput = tipPitchPid.calculate(RobotContainer.RC().sensors.getPitch());
    nt_pitch_factor.setDouble(tipPitchPidOutput);

    xSpeed = 0.0; //tipPitchPidOutput;
    ySpeed = tipRollPidOutput;
    rot = 0.0;

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    rot = MathUtil.clamp(rot, -Constants.DriveTrain.kMaxAngularSpeed, Constants.DriveTrain.kMaxAngularSpeed);

    output_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
  }

  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("***STOPPING TIP CORRECTION MODE");
    drivetrain.stop();
  }

}
