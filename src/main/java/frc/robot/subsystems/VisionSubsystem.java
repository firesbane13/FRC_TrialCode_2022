package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public boolean findTarget() {
        boolean status = false;

        return status;
    }

    public double getTargetHorizontal() {
        double horizontalValue = 0.0;

        return horizontalValue;
    }

    public double getTargetVertical() {
        double verticalValue = 0.0;

        return verticalValue;
    }

    private boolean isValidTarget() {
        boolean status = false;

        return status;
    }

    private double getHorizontal() {
        double horizontalValue = 0.0;

        return horizontalValue;
    }

    private double getVertical() {
        double verticalValue = 0.0;

        return verticalValue;
    }

    private boolean ledOn() {
        boolean status = false;

        return status;
    }

    private boolean ledOff() {
        boolean status = false;

        return status;
    }

    private boolean ledDefault() {
        boolean status = false;

        return status;
    }
}
