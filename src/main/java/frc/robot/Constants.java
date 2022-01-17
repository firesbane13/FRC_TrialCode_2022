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
    /*************************************
     * General Robot Constants
     */
    public final static double driveGearRatio = 7.29;   // 7.29:1 gearing reduction.
    public final static double movementOfInertia = 7.5; // MOI of 7.5 kg m^2 (from CAD model).
    public final static double massOfRobot = 60.0;      // The mass of the robot is 60 kg.
    public final static double wheelRadius = 4;         // The robot uses 4" radius wheels.
    public final static double trackWidth = 28;         // The track width is 0.7112 meters.

    /*************************************
     * Drive Train Motor Controller ports
     */
    public final static int motorControllerPort00 = 0;
    public final static int motorControllerPort01 = 1;

    /********************************************
     * Drive Train Encoder Channels
     */
    public final static int encoder00ChannelA = 0;
    public final static int encoder00ChannelB = 1;
    public final static int encoder01ChannelA = 2;
    public final static int encoder01ChannelB = 3;

    public final static double encoder00PPR = 740;

    /**********************************************
     * Gyroscopes
     */

    // This one is on analog ports.
    public final static int gyro00Port00 = 0;

    /**********************************************
     * Simulation Variables
     */
    public final static int driveTrainNeosPerSide = 2;
}
