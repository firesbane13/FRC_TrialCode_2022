// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;
import javax.swing.text.html.HTML.Tag;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShooterFireCommand extends CommandBase {
  private ShooterSubsystem m_shooter = null;
  private VisionSubsystem m_vision = null;
  private DriveTrainSubsystem m_driveTrain = null;

  /** Creates a new ShooterCommand. */
  public ShooterFireCommand(
    ShooterSubsystem shooterSubsystem,
    DriveTrainSubsystem driveTrainSubsystem,
    VisionSubsystem visionSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooterSubsystem;
    m_driveTrain = driveTrainSubsystem;
    m_vision = visionSubsystem;
  }

  public ShooterFireCommand(ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double shooterSpeed = 0.0;


    // Only shoot when there's a valid target in range
    if (m_vision != null && m_vision.findTarget() == true) {
      // m_vision.ledToggle();

      /**
       * Align robot to target
       */
      turnTowardsTarget();

      /**
       * TODO
       * Guess-timate how far from target robot is
       * 
       * if (close) 
       *   set Constants.Shooter.closeShooterSpeed
       * else if (mid)
       *   set Constants.Shooter.midShooterSpeed
       * else if (far)
       *   set Constants.Shooter.farShooterSpeed
       */
    } else {
      // Set shooter speed if vision subsystem is not defined.
      shooterSpeed = Constants.Shooter.closeShooterSpeed;
    }

    m_shooter.fire(shooterSpeed);
  }

  public void turnTowardsTarget() {
    final double MINSPEED = 0.35;
    final double MAXSPEED = 1.00;

    double targetX = 0.0;
    double targetTurnSpeed = 0.0;
    
    // Convert radians to power based on a K value
    targetTurnSpeed = targetX / 30;

    /**
     * Turn speed + (difference from max * minimum speed)
     */
    targetTurnSpeed = targetTurnSpeed + ((1 - targetTurnSpeed) * MINSPEED);
    
    m_driveTrain.tankDrive(targetTurnSpeed, -targetTurnSpeed);
  }

  public void moveToClosestRange() {
    /** 
     * How hard to drive forward towards the target.   
     * This value is based on the PID concept.
     */
    final double DRIVE_K = 0.05;

    /**
     * Maximum size of the target area (ta) value from the limelight.
     */
    final double DESIRED_CLOSE_TARGET_AREA = 0.26;
    final double DESIRED_MID_TARGET_AREA   = 0.13;
    final double DESIRED_FAR_TARGET_AREA   = 0.07;
    final int CLOSEST = 0;
    final int MID = 1;
    final int FAR = 2;

    // Simple speed limit
    final double MAX_DRIVE = 0.7;

    double targetArea = 0.0;

    double closeDifference = 0.0;
    double midDifference = 0.0;
    double farDifference = 0.0;

    int selectedRange = -1;

    targetArea = m_vision.getTargetArea();

    closeDifference = DESIRED_CLOSE_TARGET_AREA - targetArea;
    midDifference   = DESIRED_MID_TARGET_AREA - targetArea;
    farDifference   = DESIRED_FAR_TARGET_AREA - targetArea;

    // Find the closest optimal range
    if ( closeDifference < midDifference && closeDifference < farDifference ) {
      selectedRange = CLOSEST;
    } else if ( midDifference < farDifference ) {
      selectedRange = MID;
    } else {
      selectedRange = FAR;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
