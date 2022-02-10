package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectorSubsystem extends SubsystemBase {

    private final static int IN  = 1;
    private final static int OUT = -1;
    
    private final static int RAISE = 1;
    private final static int LOWER = -1;

    private MotorController victorSpMotorController = new VictorSP(Constants.Collector.raiseLowerPort);

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

        status = collectorPosition(speed, LOWER);

        return status;
    }

    /**
     * intake()
     * 
     * intakeIn and intakeOut call this function with a direction
     * 
     * @param speed
     * @param direction
     * @return
     */
    private boolean intake(double speed, int direction) {
        boolean status = false;

        return status;
    }

    /**
     * collectorPosition()
     * 
     * raiseCollector and lowerCollector call this function with a direction
     * 
     * @param speed
     * @param direction
     * @return
     */
    private boolean collectorPosition(double speed, int direction) {
        boolean status = false;

        victorSpMotorController.set(Constants.Collector.raiseLowerSpeed);

        return status;
    }
}
