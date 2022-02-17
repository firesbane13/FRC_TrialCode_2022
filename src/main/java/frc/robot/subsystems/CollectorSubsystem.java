package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectorSubsystem extends SubsystemBase {
    // Used to determine direction.  Does the motor take game pieces in or spit them out
    private final static int IN  = 1;
    private final static int OUT = -1;
    
    // Used to determine direction.  Does the motor raise or lower the collector
    public final static int RAISE = 1;
    public final static int LOWER = -1;
    public final static int STOP = 0;

    // Raise or Lower motor controller
    private MotorController victorSpRaiseLowerMC = new VictorSP(Constants.Collector.raiseLowerPort);
 
    // Collect or uncollect game pieces motor controller.
    private MotorController victorSpCollectorMC = new VictorSP(Constants.Collector.collectorPort);

    DigitalInput toplimitSwitch = new DigitalInput(Constants.Collector.toplimitSwitchPort);
    DigitalInput bottomLimitSwitch = new DigitalInput(Constants.Collector.bottomLimitSwitchPort);

    public CollectorSubsystem() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * intakeIn()
     * 
     * This turns the intake on so pull in game objects.
     * 
     * @param speed Speed is between 0.00 - 1.00
     * @return
     */
    public boolean intakeIn(double speed) {
        boolean status = false;

        status = intake(speed, IN);

        return status;
    }

    /**
     * intakeOut()
     * 
     * This reverses the intake to release or spit out game objects
     * out of the robot's hopper/indexer/spindexer
     * 
     * @param speed Speed is between 0.00 - 1.00
     * @return
     */
    public boolean intakeOut(double speed) {
        boolean status = false;

        status = intake(speed, OUT);

        return status;
    }

    /**
     * raiseCollector()
     * 
     * This raises the collector from a lowered position.
     * 
     * @param speed Speed is between 0.00 - 1.00
     * @return
     */
    public boolean raiseCollector(double speed) {
        boolean status = false;

        /**
         * As long as the top limit switch isn't triggered run the motor.
         */
        SmartDashboard.putNumber("Raise", speed);
        status = collectorPosition(speed, RAISE);

        return status;
    }

    /**
     * lowerCollector()
     * 
     * Lowers collector to allow for collection of game objects.
     * 
     * @param speed Speed is between 0.00 - 1.00
     * @return
     */
    public boolean lowerCollector(double speed) {
        boolean status = false;

        /**
         * As long as the bottom limit switch isn't triggered run the motor.
         */
        SmartDashboard.putNumber("Lower", speed);
        status = collectorPosition(speed, LOWER);

        return status;
    }

    /**
     * getBottomLimitSwitchState()
     * 
     * Used in the lower collector command isFinished() function
     * 
     * @return
     */
    public boolean getBottomLimitSwitchState() {
        return bottomLimitSwitch.get();
    }

    /**
     * getTopLimitSwitchState()
     * 
     * Used in the raise collector command isFinished() function
     * @return
     */
    public boolean getTopLimitSwitchState() {
        return toplimitSwitch.get();
    }

    /**
     * stopRaiseLower()
     * 
     * Stop raise/lower motor when 
     */
    public void stopRaiseLower() {
        victorSpRaiseLowerMC.set(Constants.stopMotor);
    }

    /**
     * intake()
     * 
     * intakeIn and intakeOut call this function with a direction
     * 
     * @param speed
     * @param direction // take in or spit out
     * @return
     */
    private boolean intake(double speed, int direction) {
        boolean status = false;

        victorSpCollectorMC.set(speed * direction);

        return status;
    }

    /**
     * collectorPosition()
     * 
     * raiseCollector and lowerCollector call this function with a direction
     * 
     * @param speed
     * @param direction  // take in or spit out
     * @return
     */
    private boolean collectorPosition(double speed, int direction) {
        boolean status = true;

        victorSpRaiseLowerMC.set(speed * direction);

        return status;
    }
}
