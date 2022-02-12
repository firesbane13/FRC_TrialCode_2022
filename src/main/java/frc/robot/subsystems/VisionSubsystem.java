package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    private final static Number LED_ON      = 3;
    private final static Number LED_OFF     = 1;
    private final static Number LED_DEFAULT = 0;

    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public VisionSubsystem() {
        ledOff();
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

        SmartDashboard.putBoolean("Target Found", status);

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

        SmartDashboard.putNumber("Target's Horizontal", horizontalValue);

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

        SmartDashboard.putNumber("Target's Vertical", verticalValue);

        return verticalValue;
    }

    /**
     * getTargetArea()
     * 
     * Returns the area size of the target in the Limelight's view.
     * 
     * @return
     */
    public double getTargetArea() {
        double targetArea = 0.0;

        targetArea = getArea();

        SmartDashboard.putNumber("Target's Area", targetArea);

        return targetArea;
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
        String currentState = "Off";

        if (currentMode == LED_ON) {
            status = ledOff();
            currentState = "Off";
        } else if (currentMode == LED_OFF) {
            status = ledOn();
            currentState = "On";
        }

        SmartDashboard.putString("LL LED State", currentState);

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
        int valid = 0;

        valid = limelight.getEntry("tv").getNumber(0).intValue();

        // Need to convert valid target 0 or 1 to boolean
        if (valid == 1) {
            status = true;
        }
        
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
     * getArea()
     * 
     * Get target's area from the limelight's view
     * 
     * @return
     */
    private double getArea() {
        double areaValue = 0.0;

        if (isValidTarget()) {
            areaValue = limelight.getEntry("ta").getDouble(0.0);
        }

        return areaValue;
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

    private double calculateDistance(double angleFromCenter) {
        double distance = 0.0;
        double adjustedAngle = 0.0;

        adjustedAngle  = angleFromCenter + Constants.Limelight.angleOfLimelight;

        distance = Constants.Limelight.calculatedHeight / Math.tan(adjustedAngle);

        return distance;
    }
}
