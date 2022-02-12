// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterSpeed = 0.0;
    
    SmartDashboard.putBoolean("Valid Target", m_vision.findTarget());
    SmartDashboard.putNumber("X Loc", m_vision.getTargetHorizontal());


    // Only shoot when there's a valid target in range
    if (m_vision.findTarget() == true) {
      /**
       * TODO
       * Align robot to target
       */

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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
