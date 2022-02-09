package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private static final int FIRE  = 1;
    private static final int CLEAR = -1;

    private static final int FEEDIN  = 1;
    private static final int FEEDOUT = -1;

    private TalonFX talonMotorController = new TalonFX(Constants.Shooter.talonMotorControllerId);
    private VictorSP victorMotorController = new VictorSP(Constants.Shooter.victorMotorControllerPort);

    public ShooterSubsystem() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }    

    public boolean fire(double speed) {
        boolean status = true;

        status = shooter(speed, FIRE);

        return status;
    }

    public boolean clear(double speed) {
        boolean status = true;

        status = shooter(speed, CLEAR);
        return status;
    }

    public boolean feedIn(double speed) {
        boolean status = true;

        status = feed(speed, FEEDIN);

        return status;
    }

    public boolean feedOut(double speed) {
        boolean status = true;

        status = feed(speed, FEEDOUT);

        return status;
    }

    private boolean shooter(double speed, int direction) {
        boolean status = true;

        talonMotorController.set(
            ControlMode.PercentOutput, 
            speed * direction
        );

        return status;
    }

    private boolean feed(double speed, int direction) {
        boolean status = true;

        victorMotorController.set(speed * direction);

        return status;
    }
}
