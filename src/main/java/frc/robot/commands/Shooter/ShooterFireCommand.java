// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShooterFireCommand extends CommandBase {
  private ShooterSubsystem m_shooter = null;
  private VisionSubsystem  m_vision  = null;

  /** Creates a new ShooterCommand. */
  public ShooterFireCommand(
    ShooterSubsystem shooterSubsystem,
    VisionSubsystem  visionSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = shooterSubsystem;
    this.m_vision  = visionSubsystem;

    addRequirements(shooterSubsystem);
    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Runs when the command is triggered.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetDistance = this.m_vision.calculateDistance();
    double shooterSpeed = this.m_shooter.calculateSpeed(targetDistance);

    this.m_shooter.fire(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Runs when interrupted or when isFinished == true
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
