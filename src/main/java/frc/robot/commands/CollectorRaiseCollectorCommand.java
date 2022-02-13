// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;

public class CollectorRaiseCollectorCommand extends CommandBase {
  private CollectorSubsystem m_collector;

  /** Creates a new CollectorRaiseCollectorCommand. */
  public CollectorRaiseCollectorCommand(CollectorSubsystem collectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_collector = collectorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_collector.raiseCollector(Constants.Collector.raiseLowerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean status = m_collector.getTopLimitSwitchState();

    // Stop motor when limit switch pressed
    if (status) {
      m_collector.stopRaiseLower();
    }

    // Finish when touching limit switch.
    return status;
  }
}
