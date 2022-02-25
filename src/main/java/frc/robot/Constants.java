// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static double stopMotor = 0.0;

    public final class Robot {
        /*************************************
         * General Robot Constants
         */
        public final static double wheelRadius        = 8.0;       // The robot uses 8" radius wheels.
        public final static double wheelCircumference = ((2 * Math.PI) * Constants.Robot.wheelRadius);

        public final static double robotRadius        = 48.0;      // Robot radius in inches.

        public final static double driveGearRatio    = 7.29;    // 7.29:1 gearing reduction.
        public final static double movementOfInertia = 7.5;     // MOI of 7.5 kg m^2 (from CAD model).
        public final static double massOfRobot       = 60.0;    // The mass of the robot is 60 kg.
        public final static double trackWidth        = 28;      // The track width is 0.7112 meters.

        /**
         * WHICH ROBOT
         */
        public final static int RAPIDREACT = 1;
        public final static int MECANUMBOT = 2;
        public final static int CANNONBOT  = 3;

        public final static int selectedBot = CANNONBOT;
    }

    public final class Climber {
        /**
         * PORTS
         */
        public final static int liftMotorControllerPort    = 42;
        public final static int wenchMotorControllerPort01 = 40;
        public final static int wenchMotorControllerPort02 = 41;

        /**
         * SPEEDS
         */
        public final static double liftSpeed  = 0.5;
        public final static double climbSpeed = 0.5;
    }

    public final class Collector {
        /**
         * PORTS
         */
        public final static int collectorPort  = 5;
        public final static int raiseLowerPort = 6;

        /**
         * MOTOR SPEEDS
         */
        public final static double collectorSpeed  = 0.5;
        public final static double raiseLowerSpeed = 0.25;

        /**
         * SENSOR PORTS
         */
        public final static int toplimitSwitchPort    = 8;
        public final static int bottomLimitSwitchPort = 9;
    }

    public final class DriveTrain {
        /***********************************************
         * Drive Train Type
         */
        public final static int tankDrive    = 1;   
        public final static int mecanumDrive = 2;   // X formation
        public final static int omniDrive    = 3;   // 45 degree Omni-drive
        public final static int swerveDrive  = 4;

        public final static int driveSelected = tankDrive;

        /**
         * PORTS
         */
        /*************************************
         * CANNON BOT PORTS
         */
        public final static int motorControllerPort00 = 0;
        public final static int motorControllerPort01 = 1;
        public final static int motorControllerPort02 = 2;
        public final static int motorControllerPort03 = 3;

        /*************************************
         * MECANUM BOT PORTS
         */
        public final static int canMotorDeviceId01 = 7;
        public final static int canMotorDeviceId02 = 8;
        public final static int canMotorDeviceId03 = 5;
        public final static int canMotorDeviceId04 = 6;

        /**
         * RAPIDREACT BOT
         */
        public final static int canMotorDeviceId05 = 11;
        public final static int canMotorDeviceId06 = 12;
        public final static int canMotorDeviceId07 = 9;
        public final static int canMotorDeviceId08 = 10;
    }

    public final class Joystick {
        /**
         * USB PORTS
         */
        // Joystick Only Drive Station Ports
        public final static int tankLeftPort    = 0;
        public final static int tankRightPort   = 1;

        // Joystick Only Drive Station Ports
        public final static int secondDriverPort = 2;

        // Controller Only Drive Stations 
        public final static int firstControllerPort  = 4;
        public final static int secondControllerPort = 5;
        
        public final static int joysticks  = 1;
        public final static int controllers = 2;

        public final static int configuration = joysticks;


        /**
         * BUTTONS
         */

        /*******************
         * JOYSTICK01 - PILOT
         */
        public final static int raiseLiftBtn = 5;
        public final static int lowerLiftBtn = 3;
        public final static int climbBtn = 6;
        public final static int lowerBtn = 4;

        /*******************
         * JOYSTICK02 - CO-PILOT
         */
        public final static int fireShooterBtn = 1;
        public final static int feedShooterBtn = 2;

        public final static int clearShooterBtn = 7;
        public final static int clearFeederBtn  = 8;

        public final static int raiseLowerCollectorBtn = 5;
        public final static int collectorOnOffBtn      = 3;

        public final static int clearCollectorBtn = 9;
        public final static int clearIndexerBtn   = 10;
    }

    public final class Limelight {
        // distance = (heightOfTarget - heightOfLimelight) / tan(angleOfLimeLight + angleOfTargetFromLLCenter)
        public final static double heightOfLimelight = 34.0;
        public final static double heightOfTarget    = 104.0;
        public final static double calculatedHeight  = heightOfTarget - heightOfLimelight;
        
        public final static double angleOfLimelight  = 45.0;
    }

    public final class Sensors {
        /**********************************************
         * Gyroscopes
         */

        /**
         * PORTS
         */
        public final static int gyro00Port00 = 0;
    }

    public final class Simulation { 
        /**********************************************
         * Simulation Variables
         */
        public final static int driveTrainNeosPerSide = 2;
    }

    public final class Shooter {
        /**
         * PORTS
         */
        public final static int talonMotorControllerPort  = 44;
        public final static int victorMotorControllerPort = 4;

        /**
         * POWER/SPEED
         */
        public final static double clearShooterSpeed = 0.75;
        public final static double feederSpeed       = 0.75;
        public final static double adjustmentSpeed   = 0.25;

        /**
         * DISTANCE VARIABLES
         */
        public final static double closeDistance    = 57;  // inches
        public final static double farthestDistance = 264; // inches

        public final static double closeSpeed    = 0.5;
        public final static double farthestSpeed = 0.85;

    }
}
