/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.PIDFController;

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

    public static final double DT = 0.02; // 20ms framerate 50Hz
    public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz

    public static final double FLYWHEEL_GEAR_RATIO = 1; //random number
    public static final int NEO_COUNTS_PER_REVOLUTION = 42;
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
        public static final int PCM2 = 2;

        // drive train CANCoders
        public static final int DT_BL_CANCODER = 5;
        public static final int DT_BR_CANCODER = 6; 
        public static final int DT_FR_CANCODER = 7;
        public static final int DT_FL_CANCODER = 8;
        
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
      /*
      public static final int INTAKE = 0;
      public static final int MAGAZINE = 9; 
      */
    }
    
    // Digital IO on the RIO
    public static final class DigitalIO {
      /*
      public static final int LEFT_CHASSIS_ENCODER_A = 0;
      public static final int LEFT_CHASSIS_ENCODER_B = 1;
      public static final int MAGAZINE_GATE = 2;  
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
    public static final class PCM2 {
      //public static final int MAG_LOCK = 0;
      //public static final int MAG_UNLOCK = 1;
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

    public static final class DriverPrefs {
        public static final double VelExpo = 0.3;        // non-dim [0.0 - 1.0]
        public static final double RotationExpo = 0.9;   // non-dim [0.0 - 1.0]
        public static final double StickDeadzone = 0.05; // non-dim [0.0 - 1.0]
    }

    public static final class DriveTrain {
        // motor constraints
        public static final double motorMaxRPM = 5600;    // motor limit
        public static final double wheelDiameter = 0.3333333;   //[ft]  4" wheels

        // Constraints on speeds enforeced in DriveTrain
        public static final double kMaxSpeed = 6.0; // [ft/s]
        public static final double kMaxAngularSpeed = Math.PI; // [rad/s] 1/2 rotation per second
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
        public static final PIDFController anglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0); 
        
        // CANCoder offsets for absolure calibration - stored in the magnet offset of the CC. [degrees]
        // public static final double CC_FL_OFFSET = -99.58;
        // public static final double CC_BL_OFFSET = 90.351;
        // public static final double CC_FR_OFFSET = -173.84;
        // public static final double CC_BR_OFFSET = -27.24;

        public static final double CC_FL_OFFSET = -99.667;
        public static final double CC_BL_OFFSET = 90.43;
        public static final double CC_FR_OFFSET = -175.25;
        public static final double CC_BR_OFFSET = -28.38;

        // public static final double CC_FL_OFFSET = 0;
        // public static final double CC_BL_OFFSET = 0;
        // public static final double CC_FR_OFFSET = 0;
        // public static final double CC_BR_OFFSET = 0;


        // Kinematics model - wheel offsets from center of robot (0, 0)
        // Left Front given below, symmetry used for others (in feet)
        public static final double XwheelOffset = 10.5/12;     
        public static final double YwheelOffset = 10.5/12;

        // Gear ratios - confirmed https://www.swervedrivespecialties.com/products/mk3-swerve-module?variant=39420433203313
        public static final double kSteeringGR = 12.8;   // [mo-turns to 1 angle wheel turn]
        public static final double kDriveGR = 8.16;      // [mo-turn to 1 drive wheel turn]
    }  
}
