package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainCommand extends CommandBase {
    @SuppressWarnings( {
        "PMD.UnusedPrivateField",
        "PWD.SingularField"
    } )

    private final DriveTrainSubsystem driveTrainSubsystem;
    private Joystick leftJoystick = Robot.m_robotContainer.joystick00;


    /**
     * Creates a new DriveTrainCommand.
     * 
     * @param subsystem The subsystem used by this command.
     */
    public DriveTrainCommand(DriveTrainSubsystem subsystem) {
        this.driveTrainSubsystem = subsystem;

        // Use addRequirements() here to declare subsystem dependcies.
        addRequirements(this.driveTrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Call every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveTrainSubsystem.drive();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        /**
         * If this is true the command runs once.   If it's false the command
         * is continuous
         */
        return true;
    }
}
