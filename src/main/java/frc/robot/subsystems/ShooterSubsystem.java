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

    /**
     * fire()
     * 
     * Fire shooter.
     * 
     * @param speed
     * @return
     */
    public boolean fire(double speed) {
        boolean status = true;

        status = shooter(speed, FIRE);
        status = feedIn(Constants.Shooter.feederSpeed);

        return status;
    }

    /**
     * clear()
     * 
     * Returns game piece from shooter to feeder
     * 
     * @param speed
     * @return
     */
    public boolean clear(double speed) {
        boolean status = true;

        status = shooter(speed, CLEAR);
        return status;
    }

    /**
     * feedIn()
     * 
     * Feed game piece into the shooter motor.
     * 
     * @param speed
     * @return
     */
    public boolean feedIn(double speed) {
        boolean status = true;

        status = feed(speed, FEEDIN);

        return status;
    }

    /**
     * feedOut()
     * 
     * Return game piece to the indexer/collector
     * 
     * @param speed
     * @return
     */
    public boolean feedOut(double speed) {
        boolean status = true;

        status = feed(speed, FEEDOUT);

        return status;
    }

    /**
     * stopShooter()
     * 
     * Stop shooter motor.
     * 
     * @return
     */
    public boolean stopShooter() {
        boolean status = true;

        talonMotorController.set(
            ControlMode.PercentOutput,
            Constants.stopMotor
        );

        return status;
    }

    /**
     * stopFeeder()
     * 
     * Stop feeder motor.
     * 
     * @return
     */
    public boolean stopFeeder() {
        boolean status = true;

        victorMotorController.set(Constants.stopMotor);

        return status;
    }

    /**
     * shooter()
     * 
     * The fire and clear functions call this one with a forward or back direction.
     * 
     * @param speed
     * @param direction
     * @return
     */
    private boolean shooter(double speed, int direction) {
        boolean status = true;

        talonMotorController.set(
            ControlMode.PercentOutput, 
            speed * direction
        );

        return status;
    }

    /**
     * feed()
     * 
     * feedIn and feedOut functions call this one with a forward or back direction.
     * 
     * @param speed
     * @param direction
     * @return
     */
    private boolean feed(double speed, int direction) {
        boolean status = true;

        victorMotorController.set(speed * direction);

        return status;
    }
}
