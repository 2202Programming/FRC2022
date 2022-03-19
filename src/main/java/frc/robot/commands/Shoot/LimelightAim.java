// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Aims the robot using limelight and odometry to target

package frc.robot.commands.Shoot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class LimelightAim extends CommandBase {
  /** Creates a new LimelightShoot. */

  SwerveDrivetrain drivetrain;
  Limelight_Subsystem limelight;
  final SwerveDriveKinematics kinematics;

  // output to Swerve Drivetrain
  double rot;
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

  Rotation2d currentAngle;
  Rotation2d angleError;
  Rotation2d targetAngle;
  private Pose2d centerField = new Pose2d(27, 13.5, new Rotation2d()); //actual
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter llLimiter = new SlewRateLimiter(3);
  
  private boolean finished = false;
  private double tolerance;

  public LimelightAim(double tolerance) {
    this.drivetrain = RobotContainer.RC().drivetrain;
    // tolerance is in degrees
    this.limelight = RobotContainer.RC().limelight;
    this.kinematics = drivetrain.getKinematics();
    this.tolerance = tolerance;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    anglePid.enableContinuousInput(-Math.PI, Math.PI);
    limelightPid = new PIDController(limelight_kP, limelight_kI, limelight_kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  void calculate() {

    rot = 0;
    // set goal of angle PID to be heading (in rad) from current position to
    // centerfield
    targetAngle = getHeading2Target(drivetrain.getPose(), centerField);
    targetAngle.plus(new Rotation2d(Math.PI)); // flip since shooter is on "back" of robot, bound to -pi to +pi
    currentAngle = drivetrain.getPose().getRotation(); // from -pi to pi
    angleError = targetAngle;
    angleError.minus(currentAngle);
    anglePid.setSetpoint(targetAngle.getDegrees()); //PID was tuned in degrees already
    rot = anglePid.calculate(currentAngle.getDegrees());

    if (limelight.getTarget() && limelight.getLEDStatus()) {
      // if limelight is available, override rotation input from odometery to limelight
      limelightPid.setSetpoint(0); // always go towards the light.
      limelightPidOutput = limelightPid.calculate(limelight.getFilteredX());
      // update rotation and calulate new output-states
      rot = llLimiter.calculate(limelightPidOutput);
    }

    output_states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rot, currentAngle));

    if (Math.abs(angleError.getDegrees()) < tolerance){
      finished = true;
    }
  }
  
  // takes 2 positions, gives heading from current point to target (in degrees)
  Rotation2d getHeading2Target(Pose2d current, Pose2d target) {
    // from -PI to +PI
    return new Rotation2d(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
  }

}
