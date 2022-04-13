package frc.robot.commands.Auto;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Paths;
import frc.robot.commands.LimeLightTarget;
import frc.robot.commands.ShootBalls;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.ShootingMode;
import oi.limelightvision.limelight.frc.LimeLight;

public class Auton_FiveBall extends CommandBase {

	private static DrivetrainSubsystem m_drivetrainSubsystem;
	private static IntakeSubsystem m_intakeSubsystem;
	private static TowerSubsystem m_towerSubsystem;
	private static ShooterSubsystem m_shooterSubsystem;
    private static LimeLight m_limeLight;

	private AutoIntake m_AutoIntake;
	private RunAutoTower m_AutoTower;
	private ShootBalls m_shoot2Balls;
	private ShootBalls m_shootBall;
    private LimeLightTarget m_limeLightTarget;

	private Command m_sequence;
	private Timer m_timer;

	public Auton_FiveBall(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem,
			TowerSubsystem towerSubsystem,
			ShooterSubsystem shooterSubsystem, LimeLight limeLight) {

		m_drivetrainSubsystem = drivetrainSubsystem;
		m_intakeSubsystem = intakeSubsystem;
		m_towerSubsystem = towerSubsystem;
		m_shooterSubsystem = shooterSubsystem;
				m_limeLight = limeLight;

		m_AutoIntake = new AutoIntake(m_intakeSubsystem, m_towerSubsystem);
		m_AutoTower = new RunAutoTower(m_towerSubsystem);

		m_shoot2Balls = new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 2);
		m_shootBall = new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 1);
        m_limeLightTarget = new LimeLightTarget(m_limeLight, m_drivetrainSubsystem, m_shooterSubsystem);

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
				m_shoot2Balls,
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem,
								m_isRed ? Paths.redPaths.Hub_Ball3 : Paths.bluePaths.Hub_Ball3),
						m_AutoIntake, m_AutoTower),
				new ParallelCommandGroup(
						new FollowPath(m_drivetrainSubsystem,
								m_isRed ? Paths.redPaths.Ball3_Hub : Paths.bluePaths.Ball3_Hub),
						m_AutoIntake, m_AutoTower),
				m_shootBall,
                new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem,
								m_isRed ? Paths.redPaths.Hub_Ball4 : Paths.bluePaths.Hub_Ball4),
						m_AutoIntake, m_AutoTower),
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem,
								m_isRed ? Paths.redPaths.Ball4_Tarmac : Paths.bluePaths.Ball4_Tarmac),
						m_AutoIntake, m_AutoTower),
                new InstantCommand(() -> {
                    m_shooterSubsystem.setShootingMode(ShootingMode.limeLight);
                }),
                m_limeLightTarget,
				m_shoot2Balls,
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
