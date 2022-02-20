// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignTargetCommand extends CommandBase {
  private VisionSubsystem     m_vision     = null;
  private DriveTrainSubsystem m_driveTrain = null;

  /** Creates a new VisionAlignTargetCommand. */
  public VisionAlignTargetCommand(
    VisionSubsystem visionSubsystem,
    DriveTrainSubsystem driveTrainSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = visionSubsystem;
    m_driveTrain = driveTrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnTowardsTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void turnTowardsTarget() {
    final double MINSPEED = 0.35;

    double targetX         = 0.0;
    double targetTurnSpeed = 0.0;
    
    // Angle of the target on the horizon.
    targetX = m_vision.getTargetHorizontal();

    // Convert radians to a 1 scale
    targetTurnSpeed = targetX / 30;

    /**
     * Turn speed + (difference from max * minimum speed)
     */
    targetTurnSpeed = targetTurnSpeed + ((1 - targetTurnSpeed) * MINSPEED);
    
    m_driveTrain.tankDrive(targetTurnSpeed, -targetTurnSpeed);
  }
}
