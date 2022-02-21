// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Shooter;
import frc.robot.commands.Shooter.ShooterFeedInCommand;
import frc.robot.commands.Shooter.ShooterFireCommand;
import frc.robot.commands.Shooter.ShooterStopFeederCommand;
import frc.robot.commands.Shooter.ShooterStopShooterCommand;
import frc.robot.commands.Vision.VisionAlignTargetCommand;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommands01 extends SequentialCommandGroup {
  /** Creates a new AutonomousCommands01. */
  public AutonomousCommands01(
    DriveTrainSubsystem driveTrainSubsystem,
    ShooterSubsystem    shooterSubsystem,
    VisionSubsystem     visionSubsystem,
    CollectorSubsystem  collectorSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousDriveCommand(driveTrainSubsystem).withTimeout(5),
      new VisionAlignTargetCommand(visionSubsystem, driveTrainSubsystem).withTimeout(5),
      new ShooterFireCommand(shooterSubsystem, visionSubsystem).withTimeout(5),
      new ShooterFeedInCommand(shooterSubsystem).withTimeout(5),
      new ShooterStopFeederCommand(shooterSubsystem),
      new ShooterStopShooterCommand(shooterSubsystem)
    );
  }
}
