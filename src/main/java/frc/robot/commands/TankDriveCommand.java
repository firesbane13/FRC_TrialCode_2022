package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;

public class TankDriveCommand extends CommandBase {
    @SuppressWarnings( {
        "PMD.UnusedPrivateField",
        "PWD.SingularField"
    } )

    private final TankDriveSubsystem tankDriveSubsystem;

    /**
     * Creates a new TankDriveCommand.
     * 
     * @param subsystem The subsystem used by this command.
     */
    public TankDriveCommand(TankDriveSubsystem subsystem) {
        this.tankDriveSubsystem = subsystem;

        // Use addRequirements() here to declare subsystem dependcies.
        addRequirements(this.tankDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Call every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tankDriveSubsystem.drive();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
