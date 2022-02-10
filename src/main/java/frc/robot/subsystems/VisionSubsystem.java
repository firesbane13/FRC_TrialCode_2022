package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final static Number LED_ON      = 3;
    private final static Number LED_OFF     = 1;
    private final static Number LED_DEFAULT = 0;

    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * findTarget() 
     * 
     * Used to find target
     * 
     * @return boolean Target found.
     */
    public boolean findTarget() {
        boolean status = false;

        status = isValidTarget();

        return status;
    }

    /**
     * getTargetHorizontal()
     * 
     * Get the horizontal value of the target.
     * 
     * @return double Horizontal value of the target (-29.8 to 29.8)
     */
    public double getTargetHorizontal() {
        double horizontalValue = 0.0;

        horizontalValue = getHorizontal();

        return horizontalValue;
    }

    /**
     * getTargetVertical()
     * 
     * Get the vertical value of the target.
     * 
     * @return
     */
    public double getTargetVertical() {
        double verticalValue = 0.0;

        verticalValue = getVertical();

        return verticalValue;
    }

    /**
     * ledToggle()
     * 
     * Turn Limelight LED on or off.
     * 
     * @return boolean
     */
    public boolean ledToggle() {
        boolean status     = false;
        Number currentMode = limelight.getEntry("ledMode").getNumber(-1);

        if (currentMode == LED_ON) {
            status = ledOff();
        } else if (currentMode == LED_OFF) {
            status = ledOn();
        }

        return status;
    }

    /**
     * ledDefault()
     * 
     * Set Limelight to pipeline's default mode.
     * 
     * @return boolean
     */
    public boolean ledDefault() {
        boolean status = true;

        limelight.getEntry("ledMode").setNumber(LED_DEFAULT);

        return status;
    }

    /**
     * isValidTarget()
     * 
     * Get valid target from limelight.
     * 
     * @return boolean
     */
    private boolean isValidTarget() {
        boolean status = false;

        status = limelight.getEntry("tv").getBoolean(false);

        return status;
    }

    private double getHorizontal() {
        double horizontalValue = 0.0;

        /**
         * Only get horizontal value if there's a valid target
         */
        if (isValidTarget()) {
            horizontalValue = limelight.getEntry("tx").getDouble(0.0);
        }

        return horizontalValue;
    }

    /**
     * getVertical()
     * 
     * Get target's vertical value from the Limelight
     * 
     * @return double
     */
    private double getVertical() {
        double verticalValue = 0.0;

        /**
         * Only get vertical value if there's a valid target
         */
        if (isValidTarget()) {
            verticalValue = limelight.getEntry("ty").getDouble(0.0);
        }

        return verticalValue;
    }

    /**
     * ledOn()
     * 
     * Turn Limelight LED on
     * 
     * @return boolean
     */
    private boolean ledOn() {
        boolean status = true;

        limelight.getEntry("ledMode").setNumber(LED_ON);

        return status;
    }

    /**
     * ledOff()
     * 
     * Turn Limelight LED off.
     * 
     * @return boolean
     */
    private boolean ledOff() {
        boolean status = true;

        limelight.getEntry("ledMode").setNumber(LED_OFF);

        return status;
    }
}
