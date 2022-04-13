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
import frc.robot.commands.Auto.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.ShootingMode;
import oi.limelightvision.limelight.frc.LimeLight;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * s * the {@link Robot}
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
	public final TowerSubsystem m_towerSubsystem = new TowerSubsystem();
	public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
	public final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

	// 320, 240
	// Alliance Color

	// Joysticks
	private final XboxController operator = new XboxController(1);
	private final XboxController driver = new XboxController(0);

	private final LimeLight m_limeLight = new LimeLight();

	// A chooser for autonomous commands
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	public enum LimeLightTargetState {
		Idle,
		NoTarget,
		Targeting,
		Ready
	}

	public LimeLightTargetState m_lightTargetState = LimeLightTargetState.Idle;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	private RobotContainer() {

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		m_drivetrainSubsystem.setDefaultCommand(new DrivetrainJoystick(m_drivetrainSubsystem, driver));
		m_climberSubsystem.setDefaultCommand(new ClimberTrigers(m_climberSubsystem, operator));
		m_intakeSubsystem.setDefaultCommand(new DriverIntake(m_intakeSubsystem, m_towerSubsystem, driver));
		m_towerSubsystem.setDefaultCommand(new DriverTower(m_towerSubsystem, driver));
		// m_flywheelSubsystem.setDefaultCommand(new
		// RunFlywheelJoystick(m_flywheelSubsystem, operator));

		// Configure autonomous sendable chooser
		m_chooser.setDefaultOption("Two Ball",
				new Auton_TwoBall_A(m_drivetrainSubsystem, m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem));
		m_chooser.addOption("Two Ball B",
				new Auton_TwoBall_B(m_drivetrainSubsystem, m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem));
		m_chooser.addOption("Three Ball",
				new Auton_ThreeBall(m_drivetrainSubsystem, m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem));
		m_chooser.addOption("Four Ball",
				new Auton_FourBall(m_drivetrainSubsystem, m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem));
		m_chooser.addOption("Four Ball LL",
				new Auton_FourBall_LL(m_drivetrainSubsystem, m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem, m_limeLight));
		m_chooser.addOption("Five Ball",
				new Auton_FiveBall(m_drivetrainSubsystem, m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem, m_limeLight));
		m_chooser.addOption("One Ball",
				new Auton_OneBall_TwoBall_LL(m_drivetrainSubsystem, m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem, m_limeLight, 1));
		m_chooser.addOption("Two Ball LL",
				new Auton_OneBall_TwoBall_LL(m_drivetrainSubsystem, m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem, m_limeLight, 2));

		m_chooser.addOption("Test Path", new TestPath(m_drivetrainSubsystem));
		// m_chooser.addOption("Calibate Trackwidth", new
		// CalibateTrackwidth(m_drivetrainSubsystem, false));

		SmartDashboard.putData("Auto Mode", m_chooser);

		SmartDashboard.putData("Reset Climber", new InstantCommand(() -> {
			m_climberSubsystem.resetEncoder();
		}));
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
		// Driver
		JoystickButton driver_leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
		JoystickButton driver_rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
		JoystickButton driver_xButton = new JoystickButton(driver, XboxController.Button.kX.value);

		// driver_leftBumper.whenHeld(new OutakeBalls(m_intakeSubsystem,
		// m_towerSubsystem));
		// driver_rightBumper.whenHeld(new IntakeBalls(m_intakeSubsystem,
		// m_towerSubsystem));
		new JoystickButton(driver, XboxController.Button.kA.value).whenPressed(new InstantCommand(() -> {
			m_intakeSubsystem.togglePiston();
		}));
		driver_xButton
				.whileHeld(new LimeLightTarget(m_limeLight, m_drivetrainSubsystem, m_shooterSubsystem, driver, operator));
		// Operator
		JoystickButton operator_aButton = new JoystickButton(operator, XboxController.Button.kA.value);
		JoystickButton operator_bButton = new JoystickButton(operator, XboxController.Button.kB.value);
		JoystickButton operator_xButton = new JoystickButton(operator, XboxController.Button.kX.value);
		JoystickButton operator_yButton = new JoystickButton(operator, XboxController.Button.kY.value);
		JoystickButton operator_leftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
		JoystickButton operator_rightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
		JoystickButton operator_startButton = new JoystickButton(operator, XboxController.Button.kStart.value);
		JoystickButton operator_backButton = new JoystickButton(operator, XboxController.Button.kBack.value);

		operator_rightBumper.whileHeld(new SequentialCommandGroup(
				new InstantCommand(() -> {
					m_shooterSubsystem.setShootingMode(ShootingMode.lowGoal);
				}),
				new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 0)));

		operator_leftBumper.whileHeld(new SequentialCommandGroup(
				new InstantCommand(() -> {
					m_shooterSubsystem.setShootingMode(ShootingMode.highGoal);
				}),
				new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 0)));
		operator_bButton.whileHeld(new SequentialCommandGroup(
				new InstantCommand(() -> {
					m_shooterSubsystem.setShootingMode(ShootingMode.limeLight);
				}),
				new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 0)));
		// operator_aButton.whenHeld(new RetractClimber(m_climberSubsystem));
		// operator_startButton.whenHeld(new RelseseClimber(m_climberSubsystem));
		// operator_yButton.whenHeld(new ExtendClimber(m_climberSubsystem));
		operator_xButton.whenPressed(new ToggleClimberPiston(m_climberSubsystem, m_intakeSubsystem));

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

		// Reset odometry to the starting pose of the trajectory.
		return m_chooser.getSelected();

	}

}
