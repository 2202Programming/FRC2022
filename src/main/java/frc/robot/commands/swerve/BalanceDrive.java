package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;

//Autobalance to correct tipping

public class BalanceDrive extends CommandBase {

  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] output_states;

  //probably inverted to go opposite way of tip?
  final static double kPitchStrengthCorrectionFactor = -1.0;
  final static double kRollStrengthCorrectionFactor = -1.0;

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
    xSpeed = Math.sin(Math.toRadians(RobotContainer.RC().sensors.getAHRS().getPitch())) * kPitchStrengthCorrectionFactor;
    ySpeed = Math.sin(Math.toRadians(RobotContainer.RC().sensors.getAHRS().getRoll())) * kRollStrengthCorrectionFactor;
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
