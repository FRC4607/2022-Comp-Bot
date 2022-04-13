package frc.robot.commands.Auto;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Paths;
import frc.robot.commands.ShootBalls;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.ShootingMode;

public class Auton_FourBall extends CommandBase {

	private static DrivetrainSubsystem m_drivetrainSubsystem;
	private static IntakeSubsystem m_intakeSubsystem;
	private static TowerSubsystem m_towerSubsystem;
	private static ShooterSubsystem m_shooterSubsystem;

	private AutoIntake m_AutoIntake;
	private RunAutoTower m_AutoTower;
	private ShootBalls m_shootBalls;

	private Command m_sequence;
	private Timer m_timer;

	public Auton_FourBall(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, 
						  TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {

		m_drivetrainSubsystem = drivetrainSubsystem;
		m_intakeSubsystem = intakeSubsystem;
		m_towerSubsystem = towerSubsystem;
		m_shooterSubsystem = shooterSubsystem;

		m_AutoIntake = new AutoIntake(m_intakeSubsystem, m_towerSubsystem);
		m_AutoTower = new RunAutoTower(m_towerSubsystem);

		m_shootBalls = new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 2);

		m_timer = new Timer();
	}

	@Override
	public void initialize() {

		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable FMSInfo = inst.getTable("FMSInfo");
		NetworkTableEntry alienceColor = FMSInfo.getEntry("IsRedAlliance");
		boolean m_isRed = alienceColor.getBoolean(true);

		m_drivetrainSubsystem.setBrakeMode(true);
		m_intakeSubsystem.setIntake(true);
		m_shooterSubsystem.setShootingMode(ShootingMode.highGoal);
		m_timer.reset();
		m_timer.start();

		m_sequence = new SequentialCommandGroup(
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem,
								m_isRed ? Paths.redPaths.Start_Ball2 : Paths.bluePaths.Start_Ball2),
						m_AutoIntake, m_AutoTower),
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem,
								m_isRed ? Paths.redPaths.Ball2_Hub : Paths.bluePaths.Ball2_Hub),
						m_AutoIntake, m_AutoTower),
				m_shootBalls,
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem,
								m_isRed ? Paths.redPaths.Hub_Ball3_Ball4 : Paths.bluePaths.Hub_Ball3_Ball4),
						m_AutoIntake, m_AutoTower),
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem,
								m_isRed ? Paths.redPaths.Ball4_Hub : Paths.bluePaths.Ball4_Hub),
						m_AutoIntake, m_AutoTower),
				m_shootBalls,
				new InstantCommand(() -> {
					m_drivetrainSubsystem.setBrakeMode(false);
				}));
		m_sequence.schedule();
	}

	@Override
	public void end(boolean interrupted) {
		m_sequence.cancel();
	}

	@Override
	public boolean isFinished() {
		return m_timer.hasElapsed(15);
		// return false;
	}
}
