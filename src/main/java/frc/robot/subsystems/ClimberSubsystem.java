package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final static int EXTENDCLIMBER = 1;
    private final static int RETRACTCLIMBER = -1;

    private final static int REELIN = 1;
    private final static int UNREEL = -1;

    private final static int STOP = 0;

    private VictorSPX liftMotorController = new VictorSPX(Constants.Climber.liftMotorControllerPort);

    private VictorSPX wenchMotorController01 = new VictorSPX(Constants.Climber.wenchMotorControllerPort01);
    private VictorSPX wenchMotorController02 = new VictorSPX(Constants.Climber.wenchMotorControllerPort02);

    public ClimberSubsystem() {}

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * extendArm()
     * 
     * The idea would be to extend the climber arm.   Whether it was
     * the actual climber or if it was used to deploy a grappling hook.
     * 
     * @param speed     Speed between 0.00 - 1.00
     * @return 
     */
    public boolean raiseLifter(double speed) {
        boolean status = true;

        status = arm(speed, EXTENDCLIMBER);

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
    public boolean lowerLifter(double speed) {
        boolean status = true;

        status = arm(speed, RETRACTCLIMBER);

        return status;
    }

    /**
     * stopLifter()
     * 
     * Stops lifter motor.
     * 
     * @return
     */
    public boolean stopLifter() {
        boolean status = true;

        status = arm(Constants.stopMotor, STOP);

        return status;
    }

    /**
     * reelIn()
     * 
     * If using a grappling hook climber, then this would be used to 
     * reel in the rope or whatever is attached to pull the robot up.
     * 
     * @param speed     Speed between 0.00 - 1.00
     * @return
     */
    public boolean climb(double speed) {
        boolean status = true;

        status = climber(speed, REELIN);

        return status;
    }

    /**
     * unreel()
     * 
     * If using a grappling hook climber, then this would be used to
     * slowly lower the robot to the ground or whatever.
     * 
     * @param speed     Speed between 0.00 - 1.00
     * @return
     */
    public boolean lower(double speed) {
        boolean status = true;

        status = climber(speed, UNREEL);
        return status;
    }


    /**
     * stopClimber()
     * 
     * Stops the climber motor.
     * 
     * @return
     */
    public boolean stopClimber() {
        boolean status = true;

        status = climber(Constants.stopMotor, STOP);

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
     * @param speed     Speed between 0.00 - 1.00
     * @param direction Constant of 1 (extends) or -1 (retracts)
     * @return
     */
    private boolean arm(double speed, int direction) {
        boolean status = false;

        this.liftMotorController.set(ControlMode.PercentOutput, speed * direction);

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
     * @param speed     Speed between 0.00 - 1.00
     * @param direction Constant of 1 (reel in) or -1 (unreel)
     * @return
     */
    private boolean climber(double speed, int direction) {
        boolean status = false;
        double calculatedSpeed = speed * direction;

        this.wenchMotorController01.set(ControlMode.PercentOutput, calculatedSpeed);
        this.wenchMotorController02.set(ControlMode.PercentOutput, calculatedSpeed);

        return status;
    }
}