package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class TankDriveSubsystem extends SubsystemBase {
    /**********************************************
     * 4 Wheels.   At this point it could be Omni, Mecanum, or Tank Drive
     * 
     * Omniwheels and Mecanum are controlled separately so the controllers
     * are defined separately.
     * 
     * Tank wheels are initially defined separately, but then grouped to a side
     * of the robot.   THe MotorControllerGroups below group the motor controllers
     * for each side together.
     */
    private MotorController motorController00 = new PWMSparkMax(Constants.motorControllerPort00);
    private MotorController motorController01 = new PWMSparkMax(Constants.motorControllerPort01);
    private MotorController motorController02 = new PWMSparkMax(Constants.motorControllerPort02);
    private MotorController motorController03 = new PWMSparkMax(Constants.motorControllerPort03);

    private MotorController[] leftMotorControllers = {
      this.motorController00,
      this.motorController01
    };

    private MotorController[] rightMotorControllers = {
      this.motorController02,
      this.motorController03
    };

    private MotorControllerGroup leftMotors = new MotorControllerGroup(this.leftMotorControllers);

    private MotorControllerGroup rightMotors = new MotorControllerGroup(this.rightMotorControllers);

    /************************************************
     * Drive Train Encoders
     * 
     * Encoders need two ports/channels on the DIO portion of the roboRIO.
     * 
     * See https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html
     * for explanation.
     */
    private Encoder encoder00 = new Encoder(
        Constants.encoder00ChannelA, 
        Constants.encoder00ChannelB
    );
    private Encoder encoder01 = new Encoder(
        Constants.encoder01ChannelA, 
        Constants.encoder01ChannelB
    );
    private Encoder encoder02 = new Encoder(
        Constants.encoder02ChannelA, 
        Constants.encoder02ChannelB
    );
    private Encoder encoder03 = new Encoder(
        Constants.encoder03ChannelA, 
        Constants.encoder03ChannelB
    );

    private EncoderSim encoderSim00 = new EncoderSim(this.encoder00);
    private EncoderSim encoderSim01 = new EncoderSim(this.encoder01);
    private EncoderSim encoderSim02 = new EncoderSim(this.encoder02);
    private EncoderSim encoderSim03 = new EncoderSim(this.encoder03);

    // Mostly used for simulation
    private AnalogGyro gyro00 = new AnalogGyro(Constants.gyro00Port00);
    
    private AnalogGyroSim gyroSim00 = new AnalogGyroSim(this.gyro00);


    // Create the simulation model of our drivetrain.
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(Constants.driveTrainNeosPerSide),    // 2 NEO motors on each side of the drivetrain.
        Constants.driveGearRatio,       // 7.29:1 gearing reduction.
        Constants.movementOfInertia,    // MOI of 7.5 kg m^2 (from CAD model).
        Constants.massOfRobot,          // The mass of the robot is 60 kg.
        Units.inchesToMeters(Constants.wheelRadius),    // The robot uses 4" radius wheels.
        Units.inchesToMeters(Constants.trackWidth),     // The track width is 0.7112 meters.

        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
        // l and r position: 0.005 m
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    /** Creates a new TankDriveSubsystem Object */
    public TankDriveSubsystem() {
        System.out.println("TankDriveSubsystm Constructor");

        double wheelCircumference = ((2 * Math.PI) * Constants.wheelRadius);

        encoder00.setDistancePerPulse(wheelCircumference / Constants.encoder00PPR);
        encoder01.setDistancePerPulse(wheelCircumference / Constants.encoder01PPR);
        encoder02.setDistancePerPulse(wheelCircumference / Constants.encoder02PPR);
        encoder03.setDistancePerPulse(wheelCircumference / Constants.encoder03PPR);
    }

    public void drive() {
        System.out.println("TankDriveSubsystm drive");
        return;
    }

    @Override
    public void periodic() {
        System.out.println("TankDriveSubsystm periodic");
    }

    @Override
    public void simulationPeriodic() {
        System.out.println("TankDriveSubsystm simulationPeriodic");
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        m_driveSim.setInputs(leftMotors.get() * RobotController.getInputVoltage(),
                            rightMotors.get() * RobotController.getInputVoltage());

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        m_driveSim.update(0.02);

        // Update all of our sensors.
        encoderSim00.setDistance(m_driveSim.getLeftPositionMeters());
        encoderSim00.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        encoderSim01.setDistance(m_driveSim.getLeftPositionMeters());
        encoderSim01.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        encoderSim02.setDistance(m_driveSim.getRightPositionMeters());
        encoderSim02.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        encoderSim03.setDistance(m_driveSim.getRightPositionMeters());
        encoderSim03.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        gyroSim00.setAngle(-m_driveSim.getHeading().getDegrees());
    }
}
