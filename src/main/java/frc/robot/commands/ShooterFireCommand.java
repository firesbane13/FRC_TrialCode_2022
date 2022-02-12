// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShooterFireCommand extends CommandBase {
  private ShooterSubsystem m_shooter;
  private VisionSubsystem m_vision;
  private DriveTrainSubsystem m_driveTrain;

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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double shooterSpeed = 0.0;

    // Only shoot when there's a valid target in range
    if (m_vision.findTarget() == true) {
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
    }

    m_shooter.fire(shooterSpeed);
  }

  public void turnTowardsTarget() {
    // How hard to turn towards target.   This value is based on the PID concept.
    final double STEER_K = 0.03;

    double targetX = 0.0;
    double targetTurnSpeed = 0.0;
    
    System.out.println()
    // Turn towards target until near centered
    targetX = m_vision.getTargetHorizontal(); 
      // Convert radians to power based on a K value
      targetTurnSpeed = targetX * STEER_K;

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

    /**
     * Work in progress
     */
    /*
    while ( (targetArea = m_vision.getTargetArea()) {
      m_driveTrain.tankDrive(leftSpeed, rightSpeed);
    }
    */
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
