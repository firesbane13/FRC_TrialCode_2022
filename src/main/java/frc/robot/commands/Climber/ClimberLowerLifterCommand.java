// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberLowerLifterCommand extends CommandBase {
  private ClimberSubsystem m_climber;

  /** Creates a new ClimberLowerLifterCommand. */
  public ClimberLowerLifterCommand(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climberSubsystem;

    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_climber.lowerLifter(Constants.Climber.liftSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Runs every 20ms 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Runs when interrupted or when 
    this.m_climber.stopLifter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
