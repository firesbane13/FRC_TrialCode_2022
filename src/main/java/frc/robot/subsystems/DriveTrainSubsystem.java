package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrainSubsystem extends SubsystemBase {
    public final static int RIGHT = 1;
    public final static int LEFT = -1;

    /*
    public MotorController motorController00 = new Spark(Constants.DriveTrain.motorControllerPort00);
    public MotorController motorController01 = new Spark(Constants.DriveTrain.motorControllerPort01);
    public MotorController motorController02 = new Spark(Constants.DriveTrain.motorControllerPort02);
    public MotorController motorController03 = new Spark(Constants.DriveTrain.motorControllerPort03);
    */
    
    public CANSparkMax sparkMax00;
    public MotorController motorController00;

    private CANSparkMax sparkMax01;
    public MotorController motorController01;

    private CANSparkMax sparkMax02;
    public MotorController motorController02;

    private CANSparkMax sparkMax03;
    public MotorController motorController03;

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
    private RelativeEncoder encoder00;
    private RelativeEncoder encoder01;
    private RelativeEncoder encoder02;
    private RelativeEncoder encoder03;

    /**
     * Simulation Encoders
     */
    /*
    private EncoderSim encoderSim00 = new EncoderSim(this.encoder00);
    private EncoderSim encoderSim01 = new EncoderSim(this.encoder01);
    private EncoderSim encoderSim02 = new EncoderSim(this.encoder02);
    private EncoderSim encoderSim03 = new EncoderSim(this.encoder03);
    */

    /**
     * Real Drive Train
     */
    public DifferentialDrive m_drive;
    
    /**
     * Simulation Drive Train
     * 
     * Create the simulation model of our drivetrain.
     */
    /*
    public DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(Constants.Simulation.driveTrainNeosPerSide),    // 2 NEO motors on each side of the drivetrain.
        Constants.Robot.driveGearRatio,       // 7.29:1 gearing reduction.
        Constants.Robot.movementOfInertia,    // MOI of 7.5 kg m^2 (from CAD model).
        Constants.Robot.massOfRobot,          // The mass of the robot is 60 kg.
        Units.inchesToMeters(Constants.Robot.wheelRadius),    // The robot uses 4" radius wheels.
        Units.inchesToMeters(Constants.Robot.trackWidth),     // The track width is 0.7112 meters.

        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
        // l and r pzzosition: 0.005 m
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    */

    private AnalogGyro m_gyro = new AnalogGyro(Constants.Sensors.gyro00Port00);
    /*
    // Simulation gyro.
    private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    */
    /**
     * Creating my odometry object. Here,
     * our starting pose is 5 meters along the long end of the field and in the
     * center of the field along the short end, facing forward.
     */
    public DifferentialDriveOdometry m_odometry;

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.Robot.trackWidth);

    // 2D version of the field
    public Field2d m_field = new Field2d();

    /** Creates a new TankDriveSubsystem Object */
    public DriveTrainSubsystem() {
        double wheelCircumference = ((2 * Math.PI) * Constants.Robot.wheelRadius);
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
            this.encoder00 = sparkMax00.getEncoder();
            this.encoder01 = sparkMax01.getEncoder();
            this.encoder02 = sparkMax02.getEncoder();
            this.encoder03 = sparkMax03.getEncoder();
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

        /*
        encoder00.setDistancePerPulse(wheelCircumference / Constants.DriveTrain.encoder00PPR);
        encoder00.reset();

        encoder01.setDistancePerPulse(wheelCircumference / Constants.DriveTrain.encoder01PPR);
        encoder01.reset();

        encoder02.setDistancePerPulse(wheelCircumference / Constants.DriveTrain.encoder02PPR);
        encoder02.reset();

        encoder03.setDistancePerPulse(wheelCircumference / Constants.DriveTrain.encoder03PPR);
        encoder03.reset();
        */

        // Set rightMotors reversed
        rightMotors.setInverted(true);        


        m_gyro.reset();
        /*
        m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
        */

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {

        /**
         * Get my gyro angle. We are negating the value because gyros return positive
         * values as the robot turns clockwise. This is not standard convention that is
         * used by the WPILib classes.
         */
        var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

        /*
        // Update the pose
        m_pose = m_odometry.update(gyroAngle, m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        */
    }

    @Override
    public void simulationPeriodic() {
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        /*
        m_driveSim.setInputs(leftMotors.get() * RobotController.getInputVoltage(),
                            rightMotors.get() * RobotController.getInputVoltage());
        */

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        // m_driveSim.update(0.02);

        // Update all of our sensors.
        /*
        encoderSim00.setDistance(m_driveSim.getLeftPositionMeters());
        encoderSim00.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        encoderSim01.setDistance(m_driveSim.getLeftPositionMeters());
        encoderSim01.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        encoderSim02.setDistance(m_driveSim.getLeftPositionMeters());
        encoderSim02.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        encoderSim03.setDistance(m_driveSim.getLeftPositionMeters());
        encoderSim03.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        */
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
