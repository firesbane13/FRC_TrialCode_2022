// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.simple.AutomaticSimpleMatrixConvert;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ClimberClimbCommand;
import frc.robot.commands.ClimberLowerCommand;
import frc.robot.commands.ClimberLowerLifterCommand;
import frc.robot.commands.ClimberRaiseLifterCommand;
import frc.robot.commands.ClimberStopClimberCommand;
import frc.robot.commands.ClimberStopLifterCommand;
import frc.robot.commands.CollectorRaiseLowerCollectorCommand;
import frc.robot.commands.ShooterFeedInCommand;
import frc.robot.commands.ShooterFeedOutCommand;
import frc.robot.commands.ShooterFireCommand;
import frc.robot.commands.ShooterStopFeederCommand;
import frc.robot.commands.ShooterStopShooterCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
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
  private DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private ShooterSubsystem    shooterSubsystem    = new ShooterSubsystem();
  private CollectorSubsystem  collectorSubsystem  = new CollectorSubsystem();
  private VisionSubsystem     visionSubsystem     = new VisionSubsystem();
  private ClimberSubsystem    climberSubsystem    = new ClimberSubsystem();
  
  private ShooterFireCommand fireCommand = new ShooterFireCommand(
        shooterSubsystem,
        visionSubsystem
  );
  private ShooterStopShooterCommand stopShooterCommand = new ShooterStopShooterCommand(shooterSubsystem);

  private ShooterFeedInCommand feedInCommand = new ShooterFeedInCommand(shooterSubsystem);
  private ShooterFeedOutCommand feedOutCommand = new ShooterFeedOutCommand(shooterSubsystem);
  private ShooterStopFeederCommand stopFeederCommand = new ShooterStopFeederCommand(shooterSubsystem);
  
  private CollectorRaiseLowerCollectorCommand raiseLowerCollectorCommand = new CollectorRaiseLowerCollectorCommand(collectorSubsystem);  

  private ClimberClimbCommand climbCommand = new ClimberClimbCommand(climberSubsystem);
  private ClimberLowerCommand lowerCommand = new ClimberLowerCommand(climberSubsystem);
  private ClimberStopClimberCommand stopClimberCommand = new ClimberStopClimberCommand(climberSubsystem);

  private ClimberRaiseLifterCommand raiseLifterCommand = new ClimberRaiseLifterCommand(climberSubsystem);
  private ClimberLowerLifterCommand lowerLifterCommand = new ClimberLowerLifterCommand(climberSubsystem);
  private ClimberStopLifterCommand  stopLifterCommand  = new ClimberStopLifterCommand(climberSubsystem);

  /*
  private AutonomousCommand01 autonomousCommand01 = new AutonomousCommand01(driveTrainSubsystem, shooterSubsystem, visionSubsystem, collectorSubsystem);
  private AutonomousCommand02 autonomousCommand02 = new AutonomousCommand02(driveTrainSubsystem, shooterSubsystem, visionSubsystem, collectorSubsystem);
  private AutonomousCommand03 autonomousCommand03 = new AutonomousCommand03(driveTrainSubsystem, shooterSubsystem, visionSubsystem, collectorSubsystem);
  */

  /********************************************
   * Tank Drive Controls
   */
  // Controls Left Wheels
  public Joystick joystick00 = new Joystick(Constants.Joystick.tankLeftPort);
  public Joystick joystick01 = new Joystick(Constants.Joystick.tankRightPort);
  public Joystick joystick02 = new Joystick(Constants.Joystick.secondDriverPort);

  public Joystick controller00 = new Joystick(Constants.Joystick.firstControllerPort);
  public Joystick controller01 = new Joystick(Constants.Joystick.secondControllerPort);

  public JoystickButton fireBtn        = new JoystickButton(joystick02, Constants.Joystick.fireShooterBtn);
  public JoystickButton feedShooterBtn = new JoystickButton(joystick02, Constants.Joystick.feedShooterBtn);

  public JoystickButton clearShooterBtn = new JoystickButton(joystick02, Constants.Joystick.clearShooterBtn);
  public JoystickButton clearFeederBtn  = new JoystickButton(joystick02, Constants.Joystick.clearFeederBtn);

  public JoystickButton raiseLowerCollectorBtn = new JoystickButton(joystick02, Constants.Joystick.raiseLowerCollectorBtn);
  public JoystickButton collectorOnOffBtn      = new JoystickButton(joystick02, Constants.Joystick.collectorOnOffBtn);

  public JoystickButton clearCollectorBtn = new JoystickButton(joystick02, Constants.Joystick.clearCollectorBtn);
  public JoystickButton clearIndexerBtn   = new JoystickButton(joystick02, Constants.Joystick.clearIndexerBtn);

  public JoystickButton climbBtn     = new JoystickButton(joystick01, Constants.Joystick.climbBtn);
  public JoystickButton lowerBtn     = new JoystickButton(joystick01, Constants.Joystick.lowerBtn);
  public JoystickButton raiseLiftBtn = new JoystickButton(joystick01, Constants.Joystick.raiseLiftBtn);
  public JoystickButton lowerLiftBtn = new JoystickButton(joystick01, Constants.Joystick.lowerLiftBtn);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrainSubsystem.setDefaultCommand(
      new RunCommand(
        () -> 
          driveTrainSubsystem.tankDrive(
            joystick00.getY(),
            joystick01.getY()
          ),
        driveTrainSubsystem)
      );

    /*
    m_chooser.setDefaultOption("Autonomous 01", autonomousCommand01);
    m_chooser.addOption("Autonomous 02", autonomousCommand02);
    m_chooser.addOption("Autonomous 03", autonomousCommand03);

    Shuffleboard.getTab("Autonomous").add(m_chooser);
    */
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        
    fireBtn.whileHeld(fireCommand);
    fireBtn.whenReleased(stopShooterCommand);

    feedShooterBtn.whenPressed(feedInCommand);
    feedShooterBtn.whenReleased(stopFeederCommand);

    clearFeederBtn.whenPressed(feedOutCommand);
    clearFeederBtn.whenReleased(stopFeederCommand);

    // Raise or lower the collector depending on which switch is pressed.
    raiseLowerCollectorBtn.whenPressed(raiseLowerCollectorCommand);

    climbBtn.whenHeld(climbCommand);
    climbBtn.whenReleased(stopClimberCommand);

    lowerBtn.whenHeld(lowerCommand);
    lowerBtn.whenReleased(stopClimberCommand);

    raiseLiftBtn.whenHeld(raiseLifterCommand);
    raiseLiftBtn.whenReleased(stopLifterCommand);

    lowerLiftBtn.whenHeld(lowerLifterCommand);
    lowerLiftBtn.whenReleased(stopLifterCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
