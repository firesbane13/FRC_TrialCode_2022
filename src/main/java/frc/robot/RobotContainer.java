// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import frc.robot.commands.CollectorLowerCollectorCommand;
import frc.robot.commands.CollectorRaiseCollectorCommand;
import frc.robot.commands.ShooterFeedInCommand;
import frc.robot.commands.ShooterFeedOutCommand;
import frc.robot.commands.ShooterFireCommand;
import frc.robot.commands.ShooterStopFeederCommand;
import frc.robot.commands.ShooterStopShooterCommand;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private ShooterFireCommand fireCommand = new ShooterFireCommand();
  private ShooterStopShooterCommand stopShooterCommand = new ShooterStopShooterCommand();

  private ShooterFeedInCommand feedInCommand = new ShooterFeedInCommand();
  private ShooterFeedOutCommand feedOutCommand = new ShooterFeedOutCommand();
  private ShooterStopFeederCommand stopFeederCommand = new ShooterStopFeederCommand();
  /*
  private CollectorLowerCollectorCommand lowerCollectorCommand = new CollectorLowerCollectorCommand();
  private CollectorRaiseCollectorCommand raiseCollectorCommand = new CollectorRaiseCollectorCommand();
  */

  // private CollectorSubsystem m_collector = new CollectorSubsystem();

  /********************************************
   * Tank Drive Controls
   */
  // Controls Left Wheels
  public Joystick joystick00 = new Joystick(Constants.Joystick.tankLeftPort);
  public Joystick joystick01 = new Joystick(Constants.Joystick.tankRightPort);
  public Joystick joystick02 = new Joystick(Constants.Joystick.secondDriverPort);

  public Joystick controller00 = new Joystick(Constants.Joystick.firstControllerPort);
  public Joystick controller01 = new Joystick(Constants.Joystick.secondControllerPort);

  public JoystickButton fireBtn        = new JoystickButton(joystick00, Constants.Joystick.fireShooterBtn);

  /*
  public JoystickButton feedShooterBtn = new JoystickButton(joystick02, Constants.Joystick.feedShooterBtn);

  public JoystickButton clearShooterBtn = new JoystickButton(joystick02, Constants.Joystick.clearShooterBtn);
  public JoystickButton clearFeederBtn  = new JoystickButton(joystick02, Constants.Joystick.clearFeederBtn);

  public JoystickButton raiseLowerCollectorBtn = new JoystickButton(joystick02, Constants.Joystick.raiseLowerCollectorBtn);
  public JoystickButton collectorOnOffBtn      = new JoystickButton(joystick02, Constants.Joystick.collectorOnOffBtn);

  public JoystickButton clearCollectorBtn = new JoystickButton(joystick02, Constants.Joystick.clearCollectorBtn);
  public JoystickButton clearIndexerBtn   = new JoystickButton(joystick02, Constants.Joystick.clearIndexerBtn);
*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    /*
    driveTrainSubsystem.setDefaultCommand(
      new RunCommand(
        () -> 
          driveTrainSubsystem.tankDrive(
            joystick00.getY(),
            joystick01.getY()
          ),
        driveTrainSubsystem)
      );
      */
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        
    fireBtn.whenPressed(fireCommand);
    fireBtn.whenReleased(stopShooterCommand);
    /*
    feedShooterBtn.whenPressed(feedInCommand);
    feedShooterBtn.whenReleased(stopFeederCommand);

    clearFeederBtn.whenPressed(feedOutCommand);
    clearFeederBtn.whenReleased(stopFeederCommand);
    */
    /*
    raiseLowerCollectorBtn.toggleWhenPressed(
      new StartEndCommand(
        m_collector.lowerCollector(Constants.Collector.raiseLowerSpeed), 
        m_collector.raiseCollector(Constants.Collector.raiseLowerSpeed), 
        m_collector
      )
    );
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
  }
  */
}
