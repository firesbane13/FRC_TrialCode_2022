// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutonomousDriveCommand extends CommandBase {
  private DriveTrainSubsystem m_driveTrain;
  private double leftSpeed = Constants.stopMotor;
  private double rightSpeed = Constants.stopMotor;

  /** Creates a new AutonomousDriveCommand. */
  public AutonomousDriveCommand(DriveTrainSubsystem driveTrainSubsystem, double leftSpeed, double rightSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_driveTrain = driveTrainSubsystem;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_driveTrain.tankDrive(this.leftSpeed, this.rightSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Called every 20ms
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_driveTrain.tankDrive(Constants.stopMotor, Constants.stopMotor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean status = false;

    // End drive command if told to stop.
    if (this.leftSpeed == Constants.stopMotor 
      && this.rightSpeed == Constants.stopMotor
    ) {
      status = true;
    }
    
    return status;
  }
}
