// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveTrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors_Subsystem.EncoderID;

public class SwerveDrivetrain extends SubsystemBase {
  /**
   * Inversions account for rotations of the module relative to left or right side
   * of robot.
   * 
   * CANCoders are setup in Sensors and will have CCW= positve convention. Their
   * offsets are adjusted by their use in the drive train.
   */
  boolean kDriveMotorInvert_Right = true;
  boolean kAngleMotorInvert_Right = false;
  boolean kAngleCmdInvert_Right = false;
  boolean kDriveMotorInvert_Left = false;
  boolean kAngleMotorInvert_Left = false;
  boolean kAngleCmdInvert_Left = false;
  /**
   *
   * Modules are in the order of - Front Left Front Right Back Left Back Right
   * 
   * Positive x values represent moving toward the front of the robot Positive y
   * values represent moving toward the left of the robot All lengths in feet.
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveTrain.XwheelOffset, DriveTrain.YwheelOffset), // Front Left
      new Translation2d(DriveTrain.XwheelOffset, -DriveTrain.YwheelOffset), // Front Right
      new Translation2d(-DriveTrain.XwheelOffset, DriveTrain.YwheelOffset), // Back Left
      new Translation2d(-DriveTrain.XwheelOffset, -DriveTrain.YwheelOffset) // Back Right
  );
  private SwerveDriveOdometry m_odometry;
  private Pose2d m_pose;
  private SwerveModuleState[] states;

  // sensors and our mk3 modules
  private final Sensors_Subsystem sensors;
  private final Gyro gyro;
  private final SwerveModuleMK3[] modules;

  private NetworkTable table;
  private NetworkTableEntry can_utilization;
  private NetworkTableEntry busOffCount;
  private NetworkTableEntry receiveErrorCount;
  private NetworkTableEntry transmitErrorCount;
  private NetworkTableEntry txFullCount;
  private NetworkTableEntry fieldMode;
  private NetworkTableEntry currentX;
  private NetworkTableEntry currentY;
  private NetworkTableEntry currentHeading;
  private NetworkTableEntry NTDriveMode;

  public final String NT_Name = "DT"; // expose data under DriveTrain table
  private int timer;

  private boolean fieldRelativeMode = false;
  private DriveModeTypes driveMode = DriveModeTypes.robotCentric;

  public enum DriveModeTypes {
    robotCentric("Robot Centric"), fieldCentric("Field Centric"), hubCentric("Hub Centric");

    private String name;

    private DriveModeTypes(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
    }
  }

  public SwerveDrivetrain() {
    sensors = null; //RobotContainer.RC().sensors;
    gyro = null;

    var MT = CANSparkMax.MotorType.kBrushless;
    modules = new SwerveModuleMK3[] {
        // Front Left
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_FL_DRIVE, MT), new CANSparkMax(CAN.DT_FL_ANGLE, MT),
            DriveTrain.CC_FL_OFFSET, sensors.getCANCoder(EncoderID.FrontLeft), kAngleMotorInvert_Left,
            kAngleCmdInvert_Left, kDriveMotorInvert_Left, "FL"),
        // Front Right
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_FR_DRIVE, MT), new CANSparkMax(CAN.DT_FR_ANGLE, MT),
            DriveTrain.CC_FR_OFFSET, sensors.getCANCoder(EncoderID.FrontRight), kAngleMotorInvert_Right,
            kAngleCmdInvert_Right, kDriveMotorInvert_Right, "FR"),
        // Back Left
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_BL_DRIVE, MT), new CANSparkMax(CAN.DT_BL_ANGLE, MT),
            DriveTrain.CC_BL_OFFSET, sensors.getCANCoder(EncoderID.BackLeft), kAngleMotorInvert_Left,
            kAngleCmdInvert_Left, kDriveMotorInvert_Left, "BL"),
        // Back Right
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_BR_DRIVE, MT), new CANSparkMax(CAN.DT_BR_ANGLE, MT),
            DriveTrain.CC_BR_OFFSET, sensors.getCANCoder(EncoderID.BackRight), kAngleMotorInvert_Right,
            kAngleCmdInvert_Right, kDriveMotorInvert_Right, "BR") };

    m_odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
    states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    // for updating CAN status in periodic
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    can_utilization = table.getEntry("/CanUtilization");
    busOffCount = table.getEntry("/CanBusOffCount");
    receiveErrorCount = table.getEntry("/CanReceiveErrorCount");
    transmitErrorCount = table.getEntry("/CanTransmitErrorCount");
    txFullCount = table.getEntry("/CanTxError");
    NTDriveMode = table.getEntry("/DriveMode");
    currentX = table.getEntry("/Current X");
    currentY = table.getEntry("/Current Y");
    currentHeading = table.getEntry("/Current Heading");

  }

  /**
   * Method to drive the robot using joystick info.
   * 
   * Length can be meter or ft, just be consistent in field and robot wheel units.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   *                      [length/s]
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   *                      [length/s]
   * @param rot           Angular rate of the robot. [rad/s]
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) { // should be ft/s, rad/s
    // Clamp speeds
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    rot = MathUtil.clamp(rot, -Constants.DriveTrain.kMaxAngularSpeed, Constants.DriveTrain.kMaxAngularSpeed);

    //before, was using gyro.getRotation2d() for this, but that wouldn't allow for reset of heading with resetPose.
    Rotation2d currrentHeading = m_pose.getRotation(); 

    switch(driveMode) {
      case robotCentric:
        states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        break;
      case fieldCentric:
        states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
        break;
      case hubCentric:
        states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
        break;
    }

    // output the angle and velocity for each module
    // Maybe should just call setModuleStates?
    for (int i = 0; i < states.length; i++) {
      modules[i].setDesiredState(states[i]);
    }

  }

  // used for testing
  public void testDrive(double speed, double angle) {
    // output the angle and speed (meters per sec) for each module
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle))));
    }
  }

  @Override
  public void periodic() {
    // update data from each of the swerve drive modules.
    for (int i = 0; i < modules.length; i++) {
      modules[i].periodic();
    }

    // update pose
    m_pose = m_odometry.update(gyro.getRotation2d(), states);

    // updates CAN status data every 4 cycles
    timer++;
    if (timer == 5) {
      CANStatus canStatus = RobotController.getCANStatus();
      can_utilization.setDouble(canStatus.percentBusUtilization);
      busOffCount.setDouble(canStatus.busOffCount);
      receiveErrorCount.setDouble(canStatus.receiveErrorCount);
      transmitErrorCount.setDouble(canStatus.transmitErrorCount);
      txFullCount.setDouble(canStatus.txFullCount);
      NTDriveMode.setString(driveMode.toString());
      currentX.setDouble(m_pose.getX());
      currentY.setDouble(m_pose.getY());
      currentHeading.setDouble(m_pose.getRotation().getDegrees());
      timer = 0;
    }
  }

  @Override
  public void simulationPeriodic() {
    // any sim work for each module
    for (int i = 0; i < modules.length; i++) {
      // modules[i].periodic();
    }
  }

  public SwerveModuleMK3 getMK3(int modID) {
    if ((modID < 0) || (modID > modules.length - 1))
      return null;
    return modules[modID];
  }

  public void toggleFieldRealitiveMode() {
    if (fieldRelativeMode)
      fieldRelativeMode = false;
    else
      fieldRelativeMode = true;
    fieldMode.setBoolean(fieldRelativeMode);
    return;
  }

  public void driveModeCycle() {
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
  }

  public DriveModeTypes getDriveMode() {
    return driveMode;
  }

  // sets X,Y, and sets current angle (will apply gyro correction)
  public void setPose(Pose2d new_pose) {
    m_pose = new_pose;
    m_odometry.resetPosition(m_pose, gyro.getRotation2d());
  }

  // resets X,Y, and set current angle to be 0
  public void resetPose() {
    m_pose = new Pose2d(0, 0, new Rotation2d(0));
    m_odometry.resetPosition(m_pose, gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  // Sets module states and writes to modules
  public void setModuleStates(SwerveModuleState[] newStates) {
    states = newStates; // update drivetrain version of current states; used for odometery

    // output the angle and velocity for each module
    for (int i = 0; i < states.length; i++) {
      modules[i].setDesiredState(states[i]); // updates the desired state at the module level
    }
  }
}