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
        public final static double wheelRadius       = 8;       // The robot uses 8" radius wheels.
        public final static double robotRadius       = 48;      // Robot radius in inches.

        public final static double driveGearRatio    = 7.29;    // 7.29:1 gearing reduction.
        public final static double movementOfInertia = 7.5;     // MOI of 7.5 kg m^2 (from CAD model).
        public final static double massOfRobot       = 60.0;    // The mass of the robot is 60 kg.
        public final static double trackWidth        = 28;      // The track width is 0.7112 meters.

    }

    public final class Collector {
        public final static int collectorPort  = 5;
        public final static double collectorSpeed  = 0.5;

        public final static int raiseLowerPort = 6;
        public final static double raiseLowerSpeed = 0.25;
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

        /*************************************
         * Drive Train Motor Controller ports
         */
        public final static int motorControllerPort00 = 0;
        public final static int motorControllerPort01 = 1;
        public final static int motorControllerPort02 = 2;
        public final static int motorControllerPort03 = 3;

        /*************************************
         * Drive Train Motor Controller CAN device Ids
         */
        public final static int canMotorDeviceId01 = 7;
        public final static int canMotorDeviceId02 = 8;
        public final static int canMotorDeviceId03 = 5;
        public final static int canMotorDeviceId04 = 6;

        /********************************************
         * Drive Train Encoders
         */
        public final static int sameDriveEncoder = 4096;

        public final static int encoder00ChannelA = 0;
        public final static int encoder00ChannelB = 1;
        public final static int encoder00PPR = sameDriveEncoder;

        public final static int encoder01ChannelA = 2;
        public final static int encoder01ChannelB = 3;
        public final static int encoder01PPR = sameDriveEncoder;

        public final static int encoder02ChannelA = 4;
        public final static int encoder02ChannelB = 5;
        public final static int encoder02PPR = sameDriveEncoder;

        public final static int encoder03ChannelA = 6;
        public final static int encoder03ChannelB = 7;
        public final static int encoder03PPR = sameDriveEncoder;

        public final static double movementInInches00 = ( encoder00PPR / ( Robot.wheelRadius * Math.PI ) );
        public final static double movementInInches01 = ( encoder01PPR / ( Robot.wheelRadius * Math.PI ) );
        public final static double movementInInches02 = ( encoder02PPR / ( Robot.wheelRadius * Math.PI ) );
        public final static double movementInInches03 = ( encoder03PPR / ( Robot.wheelRadius * Math.PI ) );

        public final static double movementPerDegree00 = ( ( Robot.robotRadius / ( Robot.wheelRadius / 2 ) ) * encoder00PPR ) /360;
        public final static double movementPerDegree01 = ( ( Robot.robotRadius / ( Robot.wheelRadius / 2 ) ) * encoder01PPR ) /360;
        public final static double movementPerDegree02 = ( ( Robot.robotRadius / ( Robot.wheelRadius / 2 ) ) * encoder02PPR ) /360;
        public final static double movementPerDegree03 = ( ( Robot.robotRadius / ( Robot.wheelRadius / 2 ) ) * encoder03PPR ) /360;
    }

    public final class Joystick {
        // Joystick Only Drive Station Ports
        public final static int tankLeftPort    = 0;
        public final static int tankRightPort   = 1;

        // Joystick Only Drive Station Ports
        public final static int secondDriverPort = 2;

        // Controller Only Drive Stations 
        public final static int firstControllerPort  = 4;
        public final static int secondControllerPort = 5;

        public final static int xboxControllerPort = 6;

        public final static int ps4ControllerPort = 7;

        // Joystick configuration
        public final static int joysticks  = 1;
        public final static int controllers = 2;

        public final static int configuration = joysticks;

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
        public final static double heightOfLimelight = 24.0;
        public final static double heightOfTarget    = 102.0;
        public final static double calculatedHeight  = heightOfTarget - heightOfLimelight;
        
        public final static double angleOfLimelight  = 45.0;
    }

    public final class Sensors {
        /**********************************************
         * Gyroscopes
         */

        // This one is on analog ports.
        public final static int gyro00Port00 = 0;
    }

    public final class Simulation { 
        /**********************************************
         * Simulation Variables
         */
        public final static int driveTrainNeosPerSide = 2;
    }

    public final class Shooter {
        public final static int talonMotorControllerId = 44;
        public final static int victorMotorControllerPort = 4;

        public final static double closeShooterSpeed = 0.5;
        public final static double midShooterSpeed   = 0.7;
        public final static double farShooterSpeed   = 0.9;
        public final static double clearShooterSpeed = 0.5;

        public final static double feederSpeed = 0.5;

        public final static double adjustmentSpeed = 0.25;
    }
}
