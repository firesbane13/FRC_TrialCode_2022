package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final static int EXTENDCLIMBER = 1;
    private final static int RETRACTCLIMBER = -1;

    private final static int REELIN = 1;
    private final static int UNREEL = -1;

    public ClimberSubsystem() {

    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * extendArm()
     * 
     * The idea would be to extend the climber arm.   Whether it was
     * the actual climber or if it was used to deploy a grappling hook.
     * 
     * @param distance  Distance in inches
     * @param speed     Speed between 0.00 - 1.00
     * @return 
     */
    public boolean extendArm(double distance, double speed) {
        boolean status = false;

        status = arm(distance, speed, EXTENDCLIMBER);

        return status;
    }

    /**
     * retractArm()
     * 
     * The idea would be to retract the climber arm.   Whether it was
     * the actual climber or if it was used to deploy a grappling hook.
     * 
     * @param distance  Distance in inches
     * @param speed     Speed between 0.00 - 1.00
     * @return
     */
    public boolean retractArm(double distance, double speed) {
        boolean status = false;

        status = arm(distance, speed, RETRACTCLIMBER);

        return status;
    }

    /**
     * reelIn()
     * 
     * If using a grappling hook climber, then this would be used to 
     * reel in the rope or whatever is attached to pull the robot up.
     * 
     * @param distance  Distance in inchese
     * @param speed     Speed between 0.00 - 1.00
     * @return
     */
    public boolean reelIn(double distance, double speed) {
        boolean status = false;

        status = climb(distance, speed, REELIN);

        return status;
    }

    /**
     * unreel()
     * 
     * If using a grappling hook climber, then this would be used to
     * slowly lower the robot to the ground or whatever.
     * 
     * @param distance  Distance in inches
     * @param speed     Speed between 0.00 - 1.00
     * @return
     */
    public boolean unreel(double distance, double speed) {
        boolean status = false;

        status = climb(distance, speed, UNREEL);
        return status;
    }

    /**
     * arm()
     * 
     * Extends or retracts the arm based on the direction passed in.
     * 
     * This is the actual function that uses the hardware.   The other
     * functions are to make it easier to associate buttons to commands
     * to subsystem functions.   
     * 
     * @param distance  Distance in inches
     * @param speed     Speed between 0.00 - 1.00
     * @param direction Constant of 1 (extends) or -1 (retracts)
     * @return
     */
    private boolean arm(double distance, double speed, int direction) {
        boolean status = false;

        return status;
    }

    /**
     * climb()
     * 
     * Reels in or unreels robot based on the direction passed in.
     * 
     * This is the actual function that uses the hardware.   The other
     * functions are to make it easier to associate buttons to commands
     * to subsystem functions.
     * 
     * @param distance  Distance in inches
     * @param speed     Speed between 0.00 - 1.00
     * @param direction Constant of 1 (reel in) or -1 (unreel)
     * @return
     */
    private boolean climb(double distance, double speed, int direction) {
        boolean status = false;

        return status;
    }
}