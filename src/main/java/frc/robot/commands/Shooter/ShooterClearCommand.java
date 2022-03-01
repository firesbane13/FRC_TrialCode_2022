// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterClearCommand extends CommandBase {
  private ShooterSubsystem m_shooter;

  /** Creates a new ShooterClearCommand. */
  public ShooterClearCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_shooter.clear(Constants.Shooter.clearShooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Runs every 20ms
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Runs when robot is interrupted or isFinished == true
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
