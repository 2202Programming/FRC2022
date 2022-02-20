/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.PIDFController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.shooter.FlyWheel.FlyWheelConfig;

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


    public static final double DT = 0.02; // 20ms framerate 50Hz
    public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz
    public static final int NEO_COUNTS_PER_REVOLUTION = 42;

    public static final class Autonomous {
      //These values are for a red alliance start
      public static final double RED_START_A_X = 9.91; //bottom
      public static final double RED_START_A_Y = 2.92; 
      public static final double RED_START_B_X = 9.45; //middle
      public static final double RED_START_B_Y = 5.65; 
      public static final double RED_START_C_X = 8.57; //top
      public static final double RED_START_C_Y = 6.17; 

      public static final Rotation2d RED_START_A_ROT = new Rotation2d(-180);
      public static final Rotation2d RED_START_B_ROT = new Rotation2d(-180);
      public static final Rotation2d RED_START_C_ROT = new Rotation2d(-180);

      //Blue alliance start
      public static final double BLUE_START_A_X = 6.60; //top
      public static final double BLUE_START_A_Y = 5.12; 
      public static final double BLUE_START_B_X = 7.08; //middle
      public static final double BLUE_START_B_Y = 2.62; 
      public static final double BLUE_START_C_X = 7.94; //bottom
      public static final double BLUE_START_C_Y = 2.21; 

      public static final Rotation2d BLUE_START_A_ROT = new Rotation2d(-180);
      public static final Rotation2d BLUE_START_B_ROT = new Rotation2d(-180);
      public static final Rotation2d BLUE_START_C_ROT = new Rotation2d(-180);

    }
    
    /**
     * CAN bus IDs
     * 
     * Please keep in order ID order
     * 
     */
    public static final class Autonomous {
      //These values are for a red alliance start
      public static final double RED_START_A_X = 10.0;
      public static final double RED_START_A_Y = 3.08; 
      public static final double RED_START_B_X = 9.53; 
      public static final double RED_START_B_Y = 5.58; 
      public static final double RED_START_C_X = 8.63; 
      public static final double RED_START_C_Y = 6.07; 

      public static final Rotation2d RED_START_A_ROT = new Rotation2d(0.0);
      public static final Rotation2d RED_START_B_ROT = new Rotation2d(0.0);
      public static final Rotation2d RED_START_C_ROT = new Rotation2d(0.0);

      //Blue alliance start
      public static final double BLUE_START_A_X = 6.45;
      public static final double BLUE_START_A_Y = 4.88; 
      public static final double BLUE_START_B_X = 6.99; 
      public static final double BLUE_START_B_Y = 2.49; 
      public static final double BLUE_START_C_X = 7.75; 
      public static final double BLUE_START_C_Y = 2.34; 

      public static final Rotation2d BLUE_START_A_ROT = new Rotation2d(0.0);
      public static final Rotation2d BLUE_START_B_ROT = new Rotation2d(0.0);
      public static final Rotation2d BLUE_START_C_ROT = new Rotation2d(0.0);

    }
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
        public static final int INTAKE_TALON = 14;

        // Magazine motors
        public static final int MAG_TOP_WHEEL = 16;
        public static final int MAG_R_BELT = 12;
        public static final int MAG_L_BELT = 13;
        
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
        // public static final PneumaticsModuleType MODULETYPE = CTREPCM; //DELETE LATER
        // PID values
        public static PIDFController r_beltPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0);  
        public static PIDFController l_beltPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0); 
  
        
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
        public static final double kMaxSpeed = 12.0; // [ft/s]
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
        public static final PIDFController drivePIDF = new PIDFController(0.09, 0.0, 0.0, 0.08076);  
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

        //FOR BETABOT
        public static final double CC_FL_OFFSET =    -175.60; 
        public static final double CC_BL_OFFSET =    -115.40; 
        public static final double CC_FR_OFFSET =   -162.15; 
        public static final double CC_BR_OFFSET =   158.81; 

        // public static final double CC_FL_OFFSET = 0;
        // public static final double CC_BL_OFFSET = 0;
        // public static final double CC_FR_OFFSET = 0;
        // public static final double CC_BR_OFFSET = 0;


        // Kinematics model - wheel offsets from center of robot (0, 0)
        // Left Front given below, symmetry used for others (in feet)
        // Betabot is 21.516 left-right and 24.87 front-back
        public static final double XwheelOffset = (21.516/12)/2;     
        public static final double YwheelOffset = (24.87/12)/2;
        public static final double wheelCorrectionFactor = 0.9395; //measured on swervebot
        public static final double wheelDiameter = 0.3333333 * wheelCorrectionFactor;   //[ft]  4" wheels

        // Gear ratios - confirmed https://www.swervedrivespecialties.com/products/mk3-swerve-module?variant=39420433203313
        public static final double kSteeringGR = 12.8;   // [mo-turns to 1 angle wheel turn]
        public static final double kDriveGR = 8.16;      // [mo-turn to 1 drive wheel turn]
    } 
    
    public final static class MagazineSettings {
     
    }

    public static final class Shooter {
      // Power Cell info
      // public static final double PowerCellMass = 3.0 / 16.0; // lbs
      public static final double PCNominalRadius = 10 / 2.0 / 12.0; // feet - power cell
      public static final double PCEffectiveRadius = 8 / 2.0 / 12.0; // feet - compressed radius
      public static final double FlyWheelGearRatio = 1;

      /**
       * Convert Target RPM to [motor-units/100ms] 4096 Units/Rev * Target RPM * 600 =
       * velocity setpoint is in units/100ms
       */
      public static final double kRPM2Counts = 4096.0/600.0; // MU-100 (no gearing)
      public static final double kMaxMO = 1023;  // max Motor output

      // Flywheel info
      // Flywheel maxOpenLoopRPM and gear ratio are used to calculate kFF in shooter
      public static FlyWheelConfig upperFWConfig = new FlyWheelConfig();
      static {
        upperFWConfig.maxOpenLoopRPM = 2500;  // estimated from 2000 RPM test
        upperFWConfig.gearRatio = 3.0;        // upper is 5:1 (motor:fw)
        upperFWConfig.sensorPhase = false;
        upperFWConfig.inverted = true;
        upperFWConfig.flywheelRadius = 2.0 / 12.0; // feet
        upperFWConfig.pid = new PIDFController(0.08, 0.00015, 4.0, 0); // kP kI kD kFF
        upperFWConfig.pid.setIzone(1800);
      }

      public static FlyWheelConfig lowerFWConfig = new FlyWheelConfig();
      static {
        lowerFWConfig.maxOpenLoopRPM = 2500;
        lowerFWConfig.gearRatio = 3.0;         // lower fw gear 3:1  (motor:flywheel)
        lowerFWConfig.sensorPhase = false;
        lowerFWConfig.inverted = false; 
        lowerFWConfig.flywheelRadius = 2.0 / 12.0;   //feet 
        lowerFWConfig.pid = new PIDFController(0.08, 0.00015, 4.0, 0);   // kP kI kD kF 
        lowerFWConfig.pid.setIzone(1800);
      }

    }




}
