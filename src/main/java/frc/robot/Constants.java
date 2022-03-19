/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.PIDFController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelConfig;
import frc.robot.subsystems.shooter.Shooter_Subsystem.ShooterSettings;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean HAS_INTAKE  = true;
    public static final boolean HAS_SHOOTER = true;
    public static final boolean HAS_MAGAZINE = true;
    public static final boolean HAS_DRIVETRAIN = true;
    public static final double  FTperM = 3.28084;
    public static final double  MperFT = 1.0/FTperM; 

    public static final double DT = 0.02; // 20ms framerate 50Hz
    public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz
    public static final int NEO_COUNTS_PER_REVOLUTION = 42;

    public static final class Autonomous {

      //path coordinates are in meters - utility only works in meters
      public static final Pose2d startPose1 = new Pose2d(7.67,1.82,new Rotation2d(-180)); //Bottom, furthest from terminal
      public static final Pose2d startPose2 = new Pose2d(6.86,2.63,new Rotation2d(-180)); //Middle
      public static final Pose2d startPose3 = new Pose2d(6.7,5.47,Rotation2d.fromDegrees(-180)); //Top
      public static final Pose2d hubPose =    new Pose2d(8.27,4.12,new Rotation2d(0)); //Center of Hub
      public static final Pose2d testStartPose = new Pose2d(5,5,new Rotation2d(-180));
    }
    
    /**
     * CAN bus IDs
     * 
     * Please keep in order ID order
     * 
     */
  
    public static final class CAN {
        // CAN ID for non-motor devices
        public static final int PDP = 0; // this must be 0
        public static final int PCM1 = 1; // default ID for PCM
        //public static final int PCM2 = 2;

        // drive train CANCoders
        public static final int DT_BL_CANCODER = 28;
        public static final int DT_BR_CANCODER = 31; 
        public static final int DT_FR_CANCODER = 30;
        public static final int DT_FL_CANCODER = 7;

        // Shooter CAN devices
        public static final int SHOOTER_UPPER_TALON = 10;
        public static final int SHOOTER_LOWER_TALON = 11;

        //Intake CAN
        public static final int INTAKE_MTR = 14;

        // Magazine motors
        public static final int MAG_TOP_WHEEL = 16;
        public static final int MAG_R_SIDE_MTR = 12;
        public static final int MAG_L_SIDE_MTR = 13;
        
        // drive train drive / angle motors - sparkmax neo
        public static final int DT_FL_DRIVE = 20;
        public static final int DT_FL_ANGLE = 21;

        public static final int DT_BL_DRIVE = 22;
        public static final int DT_BL_ANGLE = 23;

        public static final int DT_BR_DRIVE = 24; 
        public static final int DT_BR_ANGLE = 25; 

        public static final int DT_FR_DRIVE = 26; 
        public static final int DT_FR_ANGLE = 27;

        // Whether to burn flash or not
        public static final boolean BURN_FLASH = false;

        public static final int FLYWHEEL = 0; //random number
    }

    // PWM assignments on the Rio
    public static final class PWM {
      public static final int INTAKE = 0;
      // public static final int MAGAZINE = 9; 
      
    }
    
    //Magazine constants
    public static final class Magazine{}

    // Digital IO on the RIO
    public static final class DigitalIO {
      public static final int INTAKE_GATE = 0;
      public static final int MAGAZINE_GATE1 = 1;
      public static final int MAGAZINE_GATE2 = 2;
      public static final int MAGAZINE_GATE3 = 3;
      /* 
      public static final int LEFT_CHASSIS_ENCODER_B = 1;
      public static final int MAGAZINE_GATE_PWR = 4;  
      public static final int RIGHT_CHASSIS_ENCODER_A = 5;
      public static final int RIGHT_CHASSIS_ENCODER_B = 6;
      */
    }


    // Analog IO on the RIO
    public static final class AnalogIn {
       // public static final int MAGAZINE_ANGLE = 0;
    }

    //Pnumatics control 2 -
    public static final class PCM1 {
      // Double Solenoid
      public static final int INTAKE_UP_SOLENOID_PCM = 2;   // test value
      public static final int INTAKE_DOWN_SOLENOID_PCM = 3; //test value
      public static final int POSITIONER_UP_SOLENOID_PCM = 0;   // test value
      public static final int POSITIONER_DOWN_SOLENOID_PCM = 1; // test value
      
    }

 
    public static final class RobotPhysical {
        // public static final double BUMPER_TO_LIDAR = 100; // mm 
        // public static final double LIDAR_TO_LIDAR = 348;  // mm 

        //useful if we do modeling for tracking
       // public static final double Mass = 145;  // lbs with battery and code loaded
        
    }

    /**
     * Subsystem constants
     * 
     * Maybe be used directly by the subsystem or passed as args in construction 
     * depending on the need.
     * 
     *    <subsys>.data  convention 
     */


    // public static final class LIDAR {
    //     public static final double SAMPLE_mS = 20; // in ms
       
    // }

      //Intake Constants
      public static final class Intake {
        // PID values to get copied to the hardware
        public static PIDFController r_side_mtrPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0);  
        public static PIDFController l_side_mtrPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0); 
  
        
      }
      

      //Driver Preferences
      public static final class DriverPrefs {
          public static final double VelExpo = 0.3;        // non-dim [0.0 - 1.0]
          public static final double RotationExpo = 0.9;   // non-dim [0.0 - 1.0]
          public static final double StickDeadzone = 0.05; // non-dim [0.0 - 1.0]
      }

    public static final class DriveTrain {
        // motor constraints
        public static final double motorMaxRPM = 5600;    // motor limit

        // Constraints on speeds enforeced in DriveTrain
        public static final double kMaxSpeed = 12.0*MperFT; // [m/s]
        public static final double kMaxAngularSpeed = 2*Math.PI; // [rad/s] 
        //Max neo free speed is 12.1 ft/s per specs

        /****
         * ### REMINDER - enable these once we have basics working
        // Other constraints
        public static final int smartCurrentMax = 60;  //amps in SparkMax, max setting
        public static final int smartCurrentLimit = 35; //amps in SparkMax, inital setting
        */
        // Acceleration limits
        ///public static final double slewRateMax = 2;      //sec limits adjusting slewrate 
        //public static final boolean safetyEnabled = false; 

        // SmartMax PID values [kp, ki, kd, kff] - these get sent to hardware controller
        // DEBUG - SET FF first for drive, then add KP
        // NOTE: Not sure if the pid needs MperFT or if the scaling is done with conversion factor. I think we need it. 2/28/22
        //public static final PIDFController drivePIDF = new PIDFController(0.09*MperFT, 0.0, 0.0, 0.08076*MperFT);  
        public static final PIDFController drivePIDF = new PIDFController(0.09*FTperM, 0.0, 0.0, 0.08076*FTperM);  
        public static final PIDFController anglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0); //maybe 1.0,0.0,0.1 from SDS sample code?
        
        
        // CANCoder offsets for absolure calibration - stored in the magnet offset of the CC. [degrees]
        // public static final double CC_FL_OFFSET = -99.58;
        // public static final double CC_BL_OFFSET = 90.351;
        // public static final double CC_FR_OFFSET = -173.84;
        // public static final double CC_BR_OFFSET = -27.24;
      
        /* FOR SWERVEBOT
        public static final double CC_FL_OFFSET =   -100.142; //-99.842; //  -99.667;
        public static final double CC_BL_OFFSET =    91.33; //91.83;  //   90.43;
        public static final double CC_FR_OFFSET =   -175.135; //-174.635; // -175.25;
        public static final double CC_BR_OFFSET =   -28.215; //-28.415; //  -28.38;
        */

        //FOR BETABOT - degrees
        public static final double CC_FL_OFFSET =    -175.60; 
        public static final double CC_BL_OFFSET =    -115.40; 
        public static final double CC_FR_OFFSET =   -162.15; 
        public static final double CC_BR_OFFSET =   158.81; 

        // Kinematics model - wheel offsets from center of robot (0, 0)
        // Left Front given below, symmetry used for others 
        // Betabot is 21.516" left-right and 24.87" front-back
        public static final double XwheelOffset = MperFT*(21.516/12)/2;     
        public static final double YwheelOffset = MperFT*(24.87/12)/2; 
        public static final double wheelCorrectionFactor = 1; //measured on swervebot    TODO: test after meter conversion and measurement
        public static final double wheelDiameter = 99.5 /1000.0 * wheelCorrectionFactor;   //measured 2/28/22 mm [m]

        // Gear ratios - confirmed https://www.swervedrivespecialties.com/products/mk3-swerve-module?variant=39420433203313
        public static final double kSteeringGR = 12.8;   // [mo-turns to 1 angle wheel turn]
        public static final double kDriveGR = 8.14;      // [mo-turn to 1 drive wheel turn]  //New mk4 is 8.14:1; old swerve bot was 8.16:1
    } 
    
    public final static class NTStrings {
      public final static String NT_Name_Position = "Position";
    }

    public final static class MagazineSettings {
      public final static double defaultFrontIntakeSpeed = 0.5; 
      public final static double defaultSideIntakeSpeed = 0.3; 
      public final static double defaultMagazineSpeed = 1.0;
    }

    public static final class Shooter {
      public static final double DefaultRPMTolerance = .05;  // percent of RPM
      public static final ShooterSettings DefaultSettings = new ShooterSettings(20.0, 0.0);  //ft/s, rot/s

      // Power Cell info
      // public static final double PowerCellMass = 3.0 / 16.0; // lbs
      public static final double PCNominalRadius = 10 / 2.0 / 12.0; // feet - power cell
      public static final double PCEffectiveRadius = 8 / 2.0 / 12.0; // feet - compressed radius
      
      public static final double shortVelocity = 40;
      public static final double mediumVelocity = 50;
      public static final double longVelocity = 60;

      // constraints
      public static final double kMaxFPS = 80;      //max FPS
      public static final double maxLongRage = 8; //maximum range in long distance shooting mode
      public static final double minLongRange = 1.8; //minimum range in long distance shooting mode
      public static final double maxShortRange = 2; //maximum range in short distance shooting mode
      public static final double degPerPixel = 59.6 / 320; //limelight conversion
      public static final double angleErrorTolerance = 5.0; //allowed angle error to shoot in guided shooting modes
      // Flywheel info
      // Flywheel maxOpenLoopRPM and gear ratio are used to calculate kFF in shooter
      public static FlyWheelConfig upperFWConfig = new FlyWheelConfig();
      static {
        upperFWConfig.maxOpenLoopRPM = 4000;  // estimated from 2000 RPM test
        upperFWConfig.gearRatio = 1.0;         // upper encoder:fw is 1:1 
        upperFWConfig.sensorPhase = true;
        upperFWConfig.inverted = false;
        upperFWConfig.flywheelRadius = 2 / 12.0; // feet
        upperFWConfig.pid = new PIDFController(0.12, 0.0, 4.0, 0.034); // kP kI kD kFF
        upperFWConfig.pid.setIzone(1800);
      }

      public static FlyWheelConfig lowerFWConfig = new FlyWheelConfig();
      static {
        lowerFWConfig.maxOpenLoopRPM = 4000;
        lowerFWConfig.gearRatio = 1.0;         // lower encoder:fw is 1:1
        lowerFWConfig.sensorPhase = false;
        lowerFWConfig.inverted = false; 
        lowerFWConfig.flywheelRadius = 2 / 12.0;   //feet 
        lowerFWConfig.pid = new PIDFController(0.12, 0.0, 4.0, 0.034); // kP kI kD kFF
        lowerFWConfig.pid.setIzone(1800);
      }

    }




}
