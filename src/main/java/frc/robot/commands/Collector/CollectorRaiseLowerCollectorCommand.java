// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Collector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;

public class CollectorRaiseLowerCollectorCommand extends CommandBase {
  private CollectorSubsystem m_collector;
  private int currentDirection = CollectorSubsystem.STOP;

  /** Creates a new CollectorLowerCollector. */
  public CollectorRaiseLowerCollectorCommand( CollectorSubsystem collectorSubsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_collector = collectorSubsystem;

    addRequirements(this.m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Runs once when the command is triggered.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean topSwitchStatus    = this.m_collector.getTopLimitSwitchState();
    boolean bottomSwitchStatus = this.m_collector.getBottomLimitSwitchState();

    SmartDashboard.putBoolean("Bottom switch", !bottomSwitchStatus);

    if ( ( topSwitchStatus == false 
            && this.currentDirection == CollectorSubsystem.STOP 
          ) || this.currentDirection == CollectorSubsystem.LOWER
    ) {
      // If button pressed and the top switch is triggered then lower collector
      this.m_collector.lowerCollector( Constants.Collector.raiseLowerSpeed );

      // Save direction for the command.  As it loops it will continue running that motor.
      this.currentDirection = CollectorSubsystem.LOWER;
    } else if ( ( bottomSwitchStatus == false 
                  && this.currentDirection == CollectorSubsystem.STOP
                ) || this.currentDirection == CollectorSubsystem.RAISE
    ) {    
      // If button pressed and the bottom switch is triggered then raise collector
      this.m_collector.raiseCollector( Constants.Collector.raiseLowerSpeed );

      // Save direction for the command.  As it loops it will continue running that motor.
      this.currentDirection = CollectorSubsystem.RAISE;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_collector.stopRaiseLower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // When the limit switches are tripped they are set to false.
    boolean topSwitchStatus    = this.m_collector.getTopLimitSwitchState();
    boolean bottomSwitchStatus = this.m_collector.getBottomLimitSwitchState();
 
    boolean status = false;

    // Check if and which limit switch is tripped.
    if (this.currentDirection == CollectorSubsystem.RAISE) {
      // Flip limit switch state.   When false return true because limit switch has tripped.
      status = !topSwitchStatus;
    } else if (this.currentDirection == CollectorSubsystem.LOWER) {
      // Flip limit switch state.   When false return true because limit switch has tripped.
      status = !bottomSwitchStatus;
    } else {
      status = true;
    }

    // When the top or bottom limit is triggered, set direction and stop the motor.
    if (status) {
      this.currentDirection = CollectorSubsystem.STOP;

      this.m_collector.stopRaiseLower();
    }

    return status;
  }
}
