// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final DriveTrainCommand driveTrainCommand = new DriveTrainCommand(driveTrainSubsystem);

  /********************************************
   * Tank Drive Controls
   */
  // Controls Left Wheels
  public Joystick joystick00 = new Joystick(Constants.Joystick.tankLeftPort);
  public Joystick joystick01 = new Joystick(Constants.Joystick.tankRightPort);
  public Joystick joystick02 = new Joystick(Constants.Joystick.secondDriverPort);

  // private JoystickButton driveButton = new JoystickButton(joystick00, 1);

  private final DriveTrainSubsystem m_robotDrive = new DriveTrainSubsystem();

  /*********************************************
   * XBox Controller
   */
  // private XboxController xboxController = new XboxController(0);

  /**********************************************
   * PS4 Controller
   */
  // private PS4Controller ps4Controller = new PS4Controller(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    /*
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> 
          m_robotDrive.arcadeDrive(
            joystick00.getY(),
            joystick00.getX()
          ),
          m_robotDrive)
    );
    */

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> 
          m_robotDrive.testDrive(
            joystick00.getY()
          ), 
        m_robotDrive
      )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // driveButton.whenPressed(driveTrainCommand);
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
