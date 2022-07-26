package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.util.ModMath;
import frc.robot.util.PIDFController;

public class SwerveModuleMK3 {
  public final String NT_Name = "DT"; // expose data under DriveTrain table
  // Hardware PID settings in Constants.DriveTrain PIDFController
  // PID slot for angle and drive pid on SmartMax controller
  final int kSlot = 0;

  private int frameCounter = 0;

  // // Rev devices --> no longer necessary
  // private final CANSparkMax driveMotor;
  // private final CANSparkMax angleMotor;
  // private final SparkMaxPIDController driveMotorPID;
  // private final SparkMaxPIDController angleMotorPID; // sparkmax PID can only
  // use internal NEO encoders
  // private final RelativeEncoder angleEncoder; // aka internalAngle
  // private final RelativeEncoder driveEncoder;

  // CTRE devices
  private final CANCoder absEncoder;
  private double angleCmdInvert;
  // new CTRE devices
  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX angleMotor;

  /**
   * Warning CANCoder and CANEncoder are very close in name but very different.
   * 
   * CANCoder: CTRE, absolute position mode, +/- 180 CCW= positive CANEncoder:
   * RevRobotics, relative position only, must configure to CCW based on side &
   * gearing Continous positon so postion can be greater than 180 because it can
   * "infinitely" rotate. Cannot be inverted in Brushless mode, must invert motor
   * 
   */
  // keep for debugging
  CANCoderConfiguration absEncoderConfiguration;

  // NetworkTables
  String NTPrefix;

  // measurements made every period - public so they can be pulled for network
  // tables...
  double m_internalAngle; // measured Neo unbounded [deg]
  double m_externalAngle; // measured CANCoder bounded +/-180 [deg]
  double m_velocity; // measured velocity [wheel's-units/s] [m/s]
  double m_position; // measure wheel positon for calibraiton [m]
  double m_angle_target; // desired angle unbounded [deg]
  double m_vel_target; // desired velocity [wheel's-units/s] [m/s]
  /**
   * SwerveModuleMK3 -
   * 
   * SmartMax controllers used for angle and velocity motors.
   * 
   * SmartMax Velocity mode is used to close the velocity loop. Units will match
   * the units of the drive-wheel-diameter.
   * 
   * Angle in degrees is controlled using position mode on the SmartMax. The angle
   * // deleted brushless motor parameter, no further changes needed -- talon fx =
   * // falcon 500 only (for now), always brushless
   * positon is not constrainted to +/- 180 degrees because the Neo has 32bit
   * float resolution, so we can just let the postion grow or shrink based on the
   * how many degrees we need to change. We could rotate 1000's of time without
   * going past the resolution of the SmartMax's position tracking. [deg]
   * 
   * Example: cmd_angle = 175 ==> 175 + (n * 360) where -Turns < n < Turns ==> ...
   * -545 == -185 == 175 == 535 == 895 ...
   * 
   * Minimum number of turns in one direction before we would have to consider
   * overflow: Turns = posBitResolution / encoder-counts Turns = 2^23 / (42*12.8)
   * = 15,603
   * 
   * Batteries will need changing before then.
   * 
   */
  public String myprefix;

  public SwerveModuleMK3(WPI_TalonFX driveMtr, WPI_TalonFX angleMtr, double offsetDegrees, CANCoder absEnc,
      boolean invertAngleMtr, boolean invertAngleCmd, boolean invertDrive, String prefix) {
    driveMotor = driveMtr;
    angleMotor = angleMtr;
    absEncoder = absEnc;
    myprefix = prefix;

    // Always restore factory defaults - it removes gremlins
    driveMotor.configFactoryDefault(); // resetFactoryDefault() --> configFactoryDefault()
    angleMotor.configFactoryDefault(); // resetFactoryDefault() --> configFactoryDefault()

    // account for command sign differences if needed
    angleCmdInvert = (invertAngleCmd) ? -1.0 : 1.0;
    setMagOffset(offsetDegrees);

    // Drive Motor config
    driveMotor.setInverted(invertDrive);
    driveMotor.setNeutralMode(NeutralMode.Brake); // setIdleMode(IdleMode.kBrake) --> setNeutralMode(NeutralMode.Brake)

    // No longer necessary -- PIDs are built-in, so are encoders
    // driveMotorPID = driveMotor.getPIDController();
    // driveEncoder = driveMotor.getEncoder();

    // set driveEncoder to use units of the wheelDiameter, meters
    // driveEncoder.setPositionConversionFactor(Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR); // mo-rot to
    //                                                                                                     // wheel units
    // driveEncoder.setVelocityConversionFactor((Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR) / 60.0); // mo-rpm
    //                                                                                                              // wheel
    //                                                                                                              // units

        // TODO: check gear ratio -- 2048 counts per rev in internal falcon 500s
        // TODO: check units also -- do we need to convert? There are no separate velocity / conversion factors; 
        // TODO: create separate methods for getting velocity and position instead of from encoder necessary?
      driveMotor.configSelectedFeedbackCoefficient(Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR);

    sleep(100);
    // Angle Motor config
    angleMotor.setInverted(invertAngleMtr);
    angleMotor.setNeutralMode(NeutralMode.Brake); // setIdleMode(IdleMode.kBrake) --> setNeutralMode(NeutralMode.Brake)

    // No longer necessary -- PID, encoder built in
    // angleMotorPID = angleMotor.getPIDController();
    // angleEncoder = angleMotor.getEncoder();

    // // set angle endcoder to return values in deg and deg/s
    // angleEncoder.setPositionConversionFactor(360.0 / DriveTrain.kSteeringGR); // mo-rotations to degrees
    // angleEncoder.setVelocityConversionFactor(360.0 / DriveTrain.kSteeringGR / 60.0); // rpm to deg/s

        // TODO: check gear ratio -- 2048 counts per rev in internal falcon 500s
        // TODO: check units also -- do we need to convert? There are no separate velocity / conversion factors; 
        // TODO: create separate methods for getting velocity and position instead of from encoder necessary?
      driveMotor.configSelectedFeedbackCoefficient(360.0 / DriveTrain.kSteeringGR / 60.0);

    // // SparkMax PID values
    // DriveTrain.anglePIDF.copyTo(angleMotorPID, kSlot); // position mode --> pid is now built in, bind directly to falcon 500s
    // DriveTrain.drivePIDF.copyTo(driveMotorPID, kSlot); // velocity mode --> pid is now built in, bind directly to falcon 500s

    // --> directo to motor b/c PID built in
    angleMotor.config_kP(kSlot, DriveTrain.anglePIDF.getP());
    angleMotor.config_kI(kSlot, DriveTrain.anglePIDF.getI());
    angleMotor.config_kD(kSlot, DriveTrain.anglePIDF.getD());
    angleMotor.config_kF(kSlot, DriveTrain.anglePIDF.getF());
    angleMotor.config_IntegralZone(kSlot, DriveTrain.anglePIDF.getIzone());

    driveMotor.config_kI(kSlot, DriveTrain.drivePIDF.getI());
    driveMotor.config_kD(kSlot, DriveTrain.drivePIDF.getD());
    driveMotor.config_kP(kSlot, DriveTrain.drivePIDF.getP());
    driveMotor.config_kF(kSlot, DriveTrain.drivePIDF.getF());
    driveMotor.config_IntegralZone(kSlot, DriveTrain.drivePIDF.getIzone());
    sleep(100);

    // burn the motor flash if BURN_FLASH is true in frc.robot.Constants.CAN
    if (Constants.CAN.BURN_FLASH) {
      ErrorCode angleError = angleMotor.getLastError(); // REVLibError --> ErrorCode, burnFlash() --> getLastError()
      // TODO: Is getLastError() the best alternative? I think it is but unsure. Same comment for driveMotor below
      sleep(1500); // takes 1 sec to burn per Dean

      int counter = 0;
      while (angleError.value != 0) {
        System.out.println(prefix + " angle error: " + angleError.value);
        counter++;
        if (counter > 20) {
          System.out.println("*** ERROR *** " + prefix + " Angle Motor Flash Failed.");
          break;
        }
        sleep(100);
      }
      System.out.println(myprefix + " Angle motor flash success.");

      ErrorCode driveError = driveMotor.getLastError(); // REVLibError --> ErrorCode, burnFlash() --> getLastError()
      // TODO: Is getLastError() the best alternative? I think it is but unsure. Same comment for angleMotor above
      sleep(1500); // takes 1 sec to burn per Dean
      counter = 0;
      while (driveError.value != 0) {
        System.out.println(prefix + " drive error: " + driveError.value);
        counter++;
        if (counter > 20) {
          System.out.println("*** ERROR *** " + prefix + " Drive Motor Flash Failed.");
          break;
        }
        sleep(100);
      }
      System.out.println(myprefix + " Drive motor flash success.");
    } else {
      System.out.println("Skipped burning flash.");
    }
    /*
     * setNTPrefix - causes the network table entries to be created and updated on
     * the periodic() call.
     * 
     * Use a short string to indicate which MK unit this is.
     */
    NTPrefix = "/MK3-" + prefix;
    myprefix = prefix;
    NTConfig();

    calibrate();

  }

  // PID accessor for use in Test/Tune Commands
  public void setDrivePID(PIDFController temp) {
    driveMotor.config_kI(kSlot, temp.getI());
    driveMotor.config_kD(kSlot, temp.getD());
    driveMotor.config_kP(kSlot, temp.getP());
    driveMotor.config_kF(kSlot, temp.getF());
    driveMotor.config_IntegralZone(kSlot, temp.getIzone());
  }

  public void setAnglePID(PIDFController temp) {
    angleMotor.config_kI(kSlot, temp.getI());
    angleMotor.config_kD(kSlot, temp.getD());
    angleMotor.config_kP(kSlot, temp.getP());
    angleMotor.config_kF(kSlot, temp.getF());
    angleMotor.config_IntegralZone(kSlot, temp.getIzone());
  }

  /**
   * This adjusts the absEncoder with the given offset to correct for CANCoder
   * mounting position. This value should be persistent accross power cycles.
   * 
   * Warning, we had to sleep afer setting configs before the absolute position
   * could be read in calibrate.
   * 
   * @param offsetDegrees
   */
  void setMagOffset(double offsetDegrees) {
    // adjust magnetic offset in absEncoder, measured constants.
    absEncoderConfiguration = new CANCoderConfiguration();
    absEncoder.getAllConfigs(absEncoderConfiguration);
    // if different, update
    if (offsetDegrees != absEncoderConfiguration.magnetOffsetDegrees) {
      absEncoderConfiguration.magnetOffsetDegrees = offsetDegrees;
      absEncoder.configAllSettings(absEncoderConfiguration, 50);
    }
  }

  /**
   * calibrate() - aligns Neo internal position with absolute encoder. This needs
   * to be done at power up, or when the unbounded encoder gets close to its
   * overflow point.
   */
  void calibrate() {
    // read absEncoder position, set internal angleEncoder to that value adjust for
    // cmd inversion.
    // Average a couple of samples of the absolute encoder
    double pos_deg = absEncoder.getAbsolutePosition();
    sleep(10);
    pos_deg = (pos_deg + absEncoder.getAbsolutePosition()) / 2.0;

    angleMotor.setSelectedSensorPosition(angleCmdInvert * pos_deg * 256/45); // setPosition --> setSelectedSensorPosition
    // added 256/45 for conversion of 360deg in 1 rev which has 2048 encoder counts, angleEncoder --> angleMotor b/c enc built in
    sleep(100); // sparkmax gremlins TODO: still necessary? Test
    double temp = angleMotor.getSelectedSensorPosition(); // getPosition --> getSelectedSensorPosition
    sleep(100); // sparkmax gremlins TODO: still necessary? Test

    int counter = 0;
    while (Math.abs(pos_deg - temp) > 0.1) { // keep trying to set encoder angle if it's not matching
      angleMotor.setSelectedSensorPosition(angleCmdInvert * pos_deg * 256/45); // angleEncoder --> angleMotor, now built in
      // added 256/45 for conversion of 360deg in 1 rev which has 2048 encoder counts, angleEncoder --> angleMotor b/c enc built in 
      sleep(100); // sparkmax gremlins
      temp = angleMotor.getSelectedSensorPosition() * 45/256; // angleEncoder --> angleMotor, now built in
      // added 45/256 for conversion of 360deg in 1 rev which has 2048 encoder counts, angleEncoder --> angleMotor b/c enc built in 
      sleep(100); // sparkmax gremlins TODO: still necessary? Test
      if (counter++ > 20) {
        System.out.println("*** Angle position set failed after 20 tries ***");
        break;
      }
    }

    realityCheckTalonFx(angleCmdInvert * pos_deg, temp); // it's TalonFX now, hence name change

  }

  // it's TalonFX now, hence name change
  void realityCheckTalonFx(double angle_cancoder, double internal_angle) {
    boolean result = true;

    // TODO: unsure if these checks are possible now -- no getCoefficient or anything method on the motors, alternative way?
    // if (Math.abs(
    //     driveEncoder.getPositionConversionFactor() - Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR) > 0.1) {
    //   System.out.println("*** ERROR *** " + myprefix + " position conversion factor incorrect for drive");
    //   System.out.println("Expected Position CF: " + Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR);
    //   System.out.println("Returned Position CF: " + driveEncoder.getPositionConversionFactor());
    //   result = false;
    // }
    // if (Math.abs(driveEncoder.getVelocityConversionFactor()
    //     - Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR / 60.0) > 0.1) {
    //   System.out.println("*** ERROR *** " + myprefix + " velocity conversion factor incorrect for drive");
    //   System.out.println("Expected Vel CF: " + Math.PI * DriveTrain.wheelDiameter / DriveTrain.kDriveGR / 60.0);
    //   System.out.println("Returned Vel CF: " + driveEncoder.getVelocityConversionFactor());
    //   result = false;
    // }
    // if (Math.abs(angleEncoder.getPositionConversionFactor() - (360.0 / DriveTrain.kSteeringGR)) > 0.1) {
    //   System.out.println("*** ERROR *** " + myprefix + " position conversion factor incorrect for angle");
    //   System.out.println("Expected Angle Pos CF: " + 360.0 / DriveTrain.kSteeringGR);
    //   System.out.println("Returned Angle Pos CF: " + angleEncoder.getPositionConversionFactor());
    //   result = false;
    // }
    // if (Math.abs(angleEncoder.getVelocityConversionFactor() - (360.0 / DriveTrain.kSteeringGR / 60)) > 0.1) {
    //   System.out.println("*** ERROR *** " + myprefix + " velocity conversion factor incorrect for angle");
    //   System.out.println("Expected Angle Vel CF: " + (360.0 / DriveTrain.kSteeringGR / 60));
    //   System.out.println("Returned Angle Vel CF: " + angleEncoder.getVelocityConversionFactor());
    //   result = false;
    // }

    if (Math.abs(angle_cancoder - internal_angle) > 0.1) {
      System.out.println("*** ERROR *** " + myprefix + " angle encoder save error");
      System.out.println("Expected internal angle: " + angle_cancoder);
      System.out.println("Returned internal angle: " + internal_angle);
      result = false;
    }
    if (result) {
      System.out.println(myprefix + " passed reality checks.");
    }
    return;
  }

  // _set<> for testing during bring up.
  public void _setInvertAngleCmd(boolean invert) {
    angleCmdInvert = (invert) ? -1.0 : 1.0;
    calibrate();
  }

  public void _setInvertAngleMotor(boolean invert) {
    angleMotor.setInverted(invert);
  }

  public void _setInvertDriveMotor(boolean invert) {
    driveMotor.setInverted(invert);
  }

  /**
   * setNTPrefix - causes the network table entries to be created and updated on
   * the periodic() call.
   * 
   * Use a short string to indicate which MK unit this is.
   * 
   *
   * public SwerveModuleMK3 setNTPrefix(String prefix) { NTPrefix = "/MK3-" +
   * prefix; myprefix = prefix; NTConfig(); return this; }
   */

  public String getNTPrefix() {
    return NTPrefix;
  }

  public void periodic() {
    // measure everything at same time; these get updated every cycle
    m_internalAngle = angleMotor.getSelectedSensorPosition() * angleCmdInvert * 45/256; // angleEncoder --> angleMotor, built in
    // getPosition --> getSelectedSensorPosition, added 45/256 coefficient to go from 2048 counts/rev to 360deg/rev
    m_velocity = driveMotor.getSelectedSensorVelocity() * 10 * 45/256; // driveEncoder --> driveMotor, built in
    // getVelocity --> getSelectedSensorVelocity, added 45/256 coefficient to go from 2048 counts/rev to 360deg/rev 
    // added 10 multiplier for 100ms --> 1sec therefore units in deg/sec TODO: change units?
    m_position = driveMotor.getSelectedSensorPosition(); // driveEncoder --> driveMotor, built in
    // getPosition --> getSelectedSensorPosition, added 45/256 coefficient to go from 2048 counts/rev to 360deg/rev

    // these are for human consumption, update slower
    if (frameCounter++ == 10) {
      m_externalAngle = absEncoder.getAbsolutePosition();
      NTUpdate();
      frameCounter = 0;
    }
  }

  /**
   * This is the angle being controlled, so it should be thought of as the real
   * angle of the wheel.
   * 
   * @return SmartMax/Neo internal angle (degrees)
   */
  public Rotation2d getAngleRot2d() {
    return Rotation2d.fromDegrees(m_internalAngle);
  }

  public double getAngle() {
    return m_internalAngle;
  }

  /**
   * External Angle is external to the SmartMax/Neo and is the absolute angle
   * encoder.
   * 
   * At power-up, this angle is used to calibrate the SmartMax PID controller.
   * 
   */
  public Rotation2d getAngleExternalRot2d() {
    return Rotation2d.fromDegrees(m_externalAngle);
  }

  public double getAngleExternal() {
    return m_externalAngle;
  }

  /**
   * 
   * @return velocity wheel's units [m]
   */
  public double getVelocity() {
    return m_velocity;
  }

  /**
   * 
   * @return velocity wheel's units [m]
   */
  public double getPosition() {
    return m_position;
  }

  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * 
   * @param desiredState - A SwerveModuleState representing the desired new state
   *                     of the module
   */
  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState m_state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(m_internalAngle)); // should
                                                                                                            // favor
                                                                                                            // reversing
                                                                                                            // direction
                                                                                                            // over
    // turning > 90 degrees

    state = m_state; // uncomment to use optimized angle command
    // use position control on angle with INTERNAL encoder, scaled internally for
    // degrees
    m_angle_target = m_state.angle.getDegrees();

    // figure out how far we need to move, target - current, bounded +/-180
    double delta = ModMath.delta360(m_angle_target, m_internalAngle);
    // if we aren't moving, keep the wheels pointed where they are
    if (Math.abs(m_state.speedMetersPerSecond) < .01)
      delta = 0;

    // now add that delta to unbounded Neo angle, m_internal isn't range bound
    angleMotor.set(ControlMode.Position, angleCmdInvert * (m_internalAngle + delta) * 256/45); 
    // setReference --> set -- equivalent
    // angleMotorPID --> angleMotor -- built in, ControlType.kPosition --> ControlMode.Position -- equivalent
    // flipped parameters because methods are different, added 256/45 multiplier to go from 360deg/rev to 2048count/rev

    // use velocity control
    driveMotor.set(ControlMode.Velocity, m_state.speedMetersPerSecond * 256/45);
    // setReference --> set -- equivalent
    // driveMotorPID --> driveMotor -- built in, ControlType.kPosition --> ControlMode.Position -- equivalent
    // flipped parameters because methods are different, added 256/45 multiplier to go from 360deg/rev to 2048count/rev
  }

  /**
   * Network Tables data
   * 
   * If a prefix is given for the module, NT entries will be created and updated
   * on the periodic() call.
   * 
   */
  private NetworkTable table;
  private NetworkTableEntry nte_angle;
  private NetworkTableEntry nte_external_angle;
  private NetworkTableEntry nte_velocity;
  private NetworkTableEntry nte_position;
  private NetworkTableEntry nte_angle_target;
  private NetworkTableEntry nte_vel_target;
  private NetworkTableEntry nte_motor_current;
  private NetworkTableEntry nte_applied_output;

  void NTConfig() {
    // direct networktables logging
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    nte_angle = table.getEntry(NTPrefix + "/angle");
    nte_external_angle = table.getEntry(NTPrefix + "/angle_ext");
    nte_velocity = table.getEntry(NTPrefix + "/velocity");
    nte_angle_target = table.getEntry(NTPrefix + "/angle_target");
    nte_vel_target = table.getEntry(NTPrefix + "/velocity_target");
    nte_position = table.getEntry(NTPrefix + "/position");
    nte_motor_current = table.getEntry(NTPrefix + "/motor_current");
    nte_applied_output = table.getEntry(NTPrefix + "/applied_output");
  }

  void NTUpdate() {
    if (table == null)
      return; // not initialized, punt
    nte_angle.setDouble(m_internalAngle);
    nte_external_angle.setDouble(m_externalAngle);
    nte_velocity.setDouble(m_velocity);
    nte_position.setDouble(m_position);
    nte_angle_target.setDouble(m_angle_target);
    nte_vel_target.setDouble(m_vel_target);
    nte_motor_current.setDouble(driveMotor.getStatorCurrent()); // getOutputCurrent --> getStatorCurrent
    nte_applied_output.setDouble(driveMotor.getMotorOutputPercent()); // getAppliedOutput --> getMotorOutputPercent
  }

  void sleep(long ms) {
    try {
      Thread.sleep(ms);
    } catch (Exception e) {
    }
  }

  // built in, get the motors instead?? Or TODO: Create new config of PID values?
  // SparkMaxPIDController getDrivePID() {
  //   return driveMotorPID;
  // }

  // SparkMaxPIDController getAnglePID() {
  //   return angleMotorPID;
  // }

  public void setBrakeMode() {
    driveMotor.setNeutralMode(NeutralMode.Brake); // setIdleMode(IdleMode.kBrake) --> setNeutralMode(NeutralMode.Brake)
    angleMotor.setNeutralMode(NeutralMode.Brake); // setIdleMode(IdleMode.kBrake) --> setNeutralMode(NeutralMode.Brake)
  }

  public void setCoastMode() {
    driveMotor.setNeutralMode(NeutralMode.Coast); // setIdleMode(IdleMode.kCoast) --> setNeutralMode(NeutralMode.Coast)
    angleMotor.setNeutralMode(NeutralMode.Coast); // setIdleMode(IdleMode.kCoast) --> setNeutralMode(NeutralMode.Coast)
  }
}