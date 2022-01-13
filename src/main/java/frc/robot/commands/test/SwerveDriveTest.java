package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveDriveTest extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final double angle;
  private final double speed;

  public SwerveDriveTest(SwerveDrivetrain drivetrain, double speed, double angle) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.angle = angle;
    this.speed = speed;
  }

  @Override
  public void execute() {
    drivetrain.testDrive(speed, angle); //set all modules to speed (meters per sec) and angle (degrees)
  }

}
