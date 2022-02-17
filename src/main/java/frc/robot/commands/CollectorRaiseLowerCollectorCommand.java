// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;

public class CollectorRaiseLowerCollectorCommand extends CommandBase {
  private CollectorSubsystem m_collector;

  /** Creates a new CollectorLowerCollector. */
  public CollectorRaiseLowerCollectorCommand(CollectorSubsystem collectorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_collector = collectorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean topSwitchStatus = m_collector.getTopLimitSwitchState();
    boolean bottomSwitchStatus = m_collector.getBottomLimitSwitchState();

    SmartDashboard.putBoolean("Top switch", topSwitchStatus);
    SmartDashboard.putBoolean("Bottom switch", bottomSwitchStatus);
    if ( topSwitchStatus ) {
      // If button pressed and the top switch is triggered then lower collector
      m_collector.lowerCollector(Constants.Collector.raiseLowerSpeed);
    } else if ( bottomSwitchStatus ) {    
      // If button pressed and the bottom switch is triggered then raise collector
      m_collector.raiseCollector(Constants.Collector.raiseLowerSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean topSwitchStatus = m_collector.getTopLimitSwitchState();
    boolean bottomSwitchStatus = m_collector.getBottomLimitSwitchState();

    // If top or bottom switch triggered than stop raise/lower motor
    boolean status = (topSwitchStatus || bottomSwitchStatus);

    if (status) {
      m_collector.stopRaiseLower();
    }

    return status;

  }
}
