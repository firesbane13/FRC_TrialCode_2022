package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrainSubsystem extends SubsystemBase {
    public final static int RIGHT = 1;
    public final static int LEFT = -1;
    
    public MotorController motorController00;
    public MotorController motorController01;
    public MotorController motorController02;
    public MotorController motorController03;

    private CANSparkMax sparkMax00;
    private CANSparkMax sparkMax01;
    private CANSparkMax sparkMax02;
    private CANSparkMax sparkMax03;

    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;

    /************************************************
     * Drive Train Encoders
     * 
     * Encoders need two ports/channels on the DIO portion of the roboRIO.
     * 
     * See https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html
     * for explanation.
     */
    private RelativeEncoder encoder00 = null;
    private RelativeEncoder encoder01 = null;
    private RelativeEncoder encoder02 = null;
    private RelativeEncoder encoder03 = null;

    // Encoder's resolution.
    private double encoder00PPR;
    private double encoder01PPR;
    private double encoder02PPR;
    private double encoder03PPR;

    // Each encoders' movement speed.
    private double encoder00MovementInInches;
    private double encoder01MovementInInches;
    private double encoder02MovementInInches;
    private double encoder03MovementInInches;

    // Each encoders' degree speed.
    private double encoder00MovementPerDeg;
    private double encoder01MovementPerDeg;
    private double encoder02MovementPerDeg;
    private double encoder03MovementPerDeg;

    /**
     * Real Drive Train
     */
    public DifferentialDrive m_drive;

    private AnalogGyro m_gyro = new AnalogGyro(Constants.Sensors.gyro00Port00);
   
    /**
     * Creating my odometry object. Here,
     * our starting pose is 5 meters along the long end of the field and in the
     * center of the field along the short end, facing forward.
     */
    public DifferentialDriveOdometry m_odometry;

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.Robot.trackWidth);

    /*****************************************
     * SIMULATION
     */
    // Create the simulation model of our drivetrain.
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
    7.29,                    // 7.29:1 gearing reduction.
    7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    60.0,                    // The mass of the robot is 60 kg.
    Units.inchesToMeters(3), // The robot uses 3" radius wheels.
    0.7112,                  // The track width is 0.7112 meters.

    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    // 2D version of the field
    public Field2d m_field = new Field2d();

    /** Creates a new TankDriveSubsystem Object */
    public DriveTrainSubsystem() {
        
        int selectedBot = Constants.Robot.selectedBot;

        if (selectedBot == Constants.Robot.CANNONBOT) {
            motorController00 = new Spark(Constants.DriveTrain.motorControllerPort00);
            motorController01 = new Spark(Constants.DriveTrain.motorControllerPort01);
            motorController02 = new Spark(Constants.DriveTrain.motorControllerPort02);
            motorController03 = new Spark(Constants.DriveTrain.motorControllerPort03);
        } else if (selectedBot == Constants.Robot.MECANUMBOT) {
            this.sparkMax00 = new CANSparkMax(
                Constants.DriveTrain.canMotorDeviceId01,
                MotorType.kBrushless
            );

            this.sparkMax01 = new CANSparkMax(
                Constants.DriveTrain.canMotorDeviceId02, 
                MotorType.kBrushless
            );

            this.sparkMax02 = new CANSparkMax(
                Constants.DriveTrain.canMotorDeviceId03, 
                MotorType.kBrushless
            );

            this.sparkMax03 = new CANSparkMax(
                Constants.DriveTrain.canMotorDeviceId04, 
                MotorType.kBrushless
            );
        } else if (selectedBot == Constants.Robot.RAPIDREACT) {
            this.sparkMax00 = new CANSparkMax(
                Constants.DriveTrain.canMotorDeviceId05,
                MotorType.kBrushless
            );

            this.sparkMax01 = new CANSparkMax(
                Constants.DriveTrain.canMotorDeviceId06, 
                MotorType.kBrushless
            );

            this.sparkMax02 = new CANSparkMax(
                Constants.DriveTrain.canMotorDeviceId07, 
                MotorType.kBrushless
            );

            this.sparkMax03 = new CANSparkMax(
                Constants.DriveTrain.canMotorDeviceId08, 
                MotorType.kBrushless
            );
        }

        if (Constants.Robot.selectedBot != Constants.Robot.CANNONBOT){
            this.motorController00 = sparkMax00;
            this.motorController01 = sparkMax01;
            this.motorController02 = sparkMax02;
            this.motorController03 = sparkMax03;

            /************************************************
             * Drive Train Encoders
             * 
             * Encoders need two ports/channels on the DIO portion of the roboRIO.
             * 
             * See https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html
             * for explanation.
             */
            this.encoder00 = sparkMax00.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
            this.encoder01 = sparkMax01.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
            this.encoder02 = sparkMax02.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
            this.encoder03 = sparkMax03.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);

            this.encoder00PPR = this.encoder00.getCountsPerRevolution();
            this.encoder01PPR = this.encoder01.getCountsPerRevolution();
            this.encoder02PPR = this.encoder02.getCountsPerRevolution();
            this.encoder03PPR = this.encoder03.getCountsPerRevolution();

            this.encoder00MovementInInches = this.encoder00PPR / ((2 * Constants.Robot.wheelRadius) * Math.PI);
            this.encoder01MovementInInches = this.encoder01PPR / ((2 * Constants.Robot.wheelRadius) * Math.PI);
            this.encoder02MovementInInches = this.encoder02PPR / ((2 * Constants.Robot.wheelRadius) * Math.PI);
            this.encoder03MovementInInches = this.encoder03PPR / ((2 * Constants.Robot.wheelRadius) * Math.PI);

            this.encoder00MovementPerDeg = ((Constants.Robot.robotRadius / Constants.Robot.wheelRadius) * this.encoder00PPR) / 360;
            this.encoder01MovementPerDeg = ((Constants.Robot.robotRadius / Constants.Robot.wheelRadius) * this.encoder01PPR) / 360;
            this.encoder02MovementPerDeg = ((Constants.Robot.robotRadius / Constants.Robot.wheelRadius) * this.encoder02PPR) / 360;
            this.encoder03MovementPerDeg = ((Constants.Robot.robotRadius / Constants.Robot.wheelRadius) * this.encoder03PPR) / 360;
        }

        this.leftMotors = new MotorControllerGroup(
            this.motorController00,
            this.motorController01
        );

        this.rightMotors = new MotorControllerGroup(
            this.motorController02,
            this.motorController03
        );
        
        m_drive = new DifferentialDrive(leftMotors, rightMotors);

        // Set rightMotors reversed
        rightMotors.setInverted(true);        

        m_gyro.reset();

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * tankDrive()
     * 
     * Functionality to implement tank drive.
     * 
     * @param leftSpeed
     * @param rightSpeed
     * @return
     */
    public boolean tankDrive(double leftSpeed, double rightSpeed) {
        boolean status = true;

        SmartDashboard.putNumber("leftSpeed", leftSpeed);
        SmartDashboard.putNumber("rightSpeed", rightSpeed);

        m_drive.tankDrive(leftSpeed, rightSpeed);

        return status;
    }

    /**
     * arcadeDrive()
     * 
     * A basic drive train for Mecnum and Omnidriver
     * 
     * @param speed
     * @param rotation
     * @return
     */
    public boolean arcadeDrive(double speed, double rotation) {
        boolean status = true;

        m_drive.arcadeDrive(speed, rotation);

        return status;
    }

    /**
     * moveForwardOrBack(distance, speed)
     * 
     * Meant for autonomous for moving forward or back a specified number of inches.
     * 
     * @param distance
     * @param speed
     * @return
     */
    public boolean moveForwardOrBack(double distance, double speed) {
        boolean status = false;
        double  encoder00Position = this.encoder00.getPosition();
        double  encoder01Position = this.encoder01.getPosition();
        double  encoder02Position = this.encoder02.getPosition();
        double  encoder03Position = this.encoder03.getPosition();

        

        return status;
    }

    /**
     * strafeLeftOrRight(distance, speed)
     * 
     * Meant for autonomous for moving left or right a specified number of inches.
     * This is mean for Mecnum and 45 deg Omniwheels
     * 
     * @param distance
     * @param speed
     * @return
     */
    public boolean strafeLeftOrRight(double distance, double speed) {
        boolean status = false;

        return status;
    }

    /**
     * rotateLeftOrRight(rotateAngle, speed) 
     * 
     * Meant for autonomous for rotating a specified angle in degrees
     * 
     * @param rotateAngle
     * @param speed
     * @return
     */
    public boolean rotateLeftOrRight(double rotateAngle, double speed) {
        boolean status = false;

        return status;
    }

    /**
     * turnLeftOrRight(distance, speed, direction)
     * 
     * This is used to move forward or back and turning left or right a specified 
     * number of inches.
     * 
     * Meant for autonomous.
     * 
     * @param distance
     * @param speed
     * @param direction
     * @return
     */
    public boolean turnLeftOrRight(double distance, double speed, int direction) {
        boolean status = false;

        return status;
    }

    /**
     * updateOdometry()
     * 
     * Updates the field-relative position
     */
    public void updateOdometry() {
        m_odometry.update(
            m_gyro.getRotation2d(), encoder00.getPosition(), encoder01.getPosition());
    }
}
