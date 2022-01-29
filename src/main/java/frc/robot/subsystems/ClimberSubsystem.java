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

    public boolean extendArm(double distance, double speed) {
        boolean status = false;

        status = arm(distance, speed, EXTENDCLIMBER);

        return status;
    }

    public boolean retractArm(double distance, double speed) {
        boolean status = false;

        status = arm(distance, speed, RETRACTCLIMBER);

        return status;
    }

    public boolean reelIn(double distance, double speed) {
        boolean status = false;

        status = climb(distance, speed, REELIN);

        return status;
    }

    public boolean unreel(double distance, double speed) {
        boolean status = false;

        status = climb(distance, speed, UNREEL);
        return status;
    }

    private boolean arm(double distance, double speed, int direction) {
        boolean status = false;

        return status;
    }

    private boolean climb(double distance, double speed, int direction) {
        boolean status = false;

        return status;
    }
}