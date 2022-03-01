// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    this.m_vision = visionSubsystem;
    this.m_driveTrain = driveTrainSubsystem;

    addRequirements(visionSubsystem);
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Runs when command is triggered.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnTowardsTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_driveTrain.tankDrive(Constants.stopMotor, Constants.stopMotor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void turnTowardsTarget() {
    final double MINSPEED = 0.35;
    final double Kp       = -0.02;

    double targetX         = 0.0;
    double targetTurnSpeed = 0.0;
    double headingError    = 0.0;
    
    // Angle of the target on the horizon.
    targetX = m_vision.getTargetHorizontal();
    headingError = -targetX;
    
    if (targetX > 1.0) {
      targetTurnSpeed = ((Kp * headingError) - MINSPEED);
    } else if (targetX < 1.0) {
      targetTurnSpeed = ((Kp * headingError) + MINSPEED);
    }
    
    SmartDashboard.putNumber("Align: Target X", targetX);
    SmartDashboard.putNumber("Align: Heading Err", headingError);
    SmartDashboard.putNumber("Align: Target Turn Spd", targetTurnSpeed);

    this.m_driveTrain.tankDrive(targetTurnSpeed, -targetTurnSpeed);
  }
}
