// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //public final TowerSubsystem m_towerSubsystem = new TowerSubsystem();
  public final Flywheel m_Flywheel = new Flywheel();

  // Joysticks
  private final XboxController operator = new XboxController(1);
  private final XboxController driver = new XboxController(0);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    // Smartdashboard Subsystems

    // SmartDashboard Buttons
    SmartDashboard.putData("AutonomousCommand", new AutonomousCommand(m_drivetrainSubsystem));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_Flywheel.setDefaultCommand(new RunFlywheelJoystick(m_Flywheel, driver));
    m_drivetrainSubsystem.setDefaultCommand(new DrivetrainJoystick(m_drivetrainSubsystem, driver));
    m_intakeSubsystem.setDefaultCommand(new RunIntake(m_intakeSubsystem, driver));
    
    // Configure autonomous sendable chooser
    m_chooser.setDefaultOption("AutonomousCommand", new AutonomousCommand(m_drivetrainSubsystem));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    JoystickButton operator_aButton = new JoystickButton(operator, 1);
    JoystickButton operator_bButton = new JoystickButton(operator, 2);
    JoystickButton operator_xButton = new JoystickButton(operator, 3);
    JoystickButton operator_yButton = new JoystickButton(operator, 4);

    JoystickButton driver_aButton = new JoystickButton(driver, 1);
    JoystickButton driver_bButton = new JoystickButton(driver, 2);
    JoystickButton driver_xButton = new JoystickButton(driver, 3);
    JoystickButton driver_yButton = new JoystickButton(driver, 4);
    
    driver_aButton.whenPressed( new ToggleIntake(m_intakeSubsystem) );

    // operator_aButton.whileHeld(new RunAgitator(false, m_towerSubsystem));
    // operator_bButton.whileHeld(new RunAgitator(true, m_towerSubsystem));
    // operator_xButton.whileHeld(new RunTransferWheel(false, m_towerSubsystem));
    // operator_yButton.whileHeld(new RunTransferWheel(true, m_towerSubsystem));
  }

  public XboxController getDriver() {
    return driver;
  }

  public XboxController getOperator() {
    return operator;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

}
