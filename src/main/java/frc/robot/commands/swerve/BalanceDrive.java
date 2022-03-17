package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
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


public class BalanceDrive extends CommandBase {

  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] output_states;

  double log_counter = 0;

  public BalanceDrive(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.kinematics = drivetrain.getKinematics();
  }


  @Override
  public void initialize() {
  }

  void calculate() {
    // Magnatude of roll or pitch correction dependant on degree of tip
    xSpeed = Math.sin(Math.toRadians(RobotContainer.RC().sensors.getAHRS().getPitch())) * -1.0;
    ySpeed = Math.sin(Math.toRadians(RobotContainer.RC().sensors.getAHRS().getRoll())) * -1.0;
    rot = 0;
    output_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
  }

  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

}
