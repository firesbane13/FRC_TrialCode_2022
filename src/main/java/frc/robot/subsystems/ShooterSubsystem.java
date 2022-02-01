package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private static final int FIRE  = 1;
    private static final int CLEAR = -1;
    
    public ShooterSubsystem() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }    

    public boolean fire(double speed) {
        boolean status = false;

        status = shooter(speed, FIRE);

        return status;
    }

    public boolean clear(double speed) {
        boolean status = false;

        status = shooter(speed, CLEAR);
        return status;
    }

    private boolean shooter(double speed, int direction) {
        boolean status = false;

        return status;
    }
}
