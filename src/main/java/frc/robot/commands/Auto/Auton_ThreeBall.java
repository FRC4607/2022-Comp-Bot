package frc.robot.commands.Auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.ShootBalls;
import frc.robot.subsystems.*;

public class Auton_ThreeBall extends CommandBase {
	private static CommandScheduler m_commandScheduler;

	private static DrivetrainSubsystem m_drivetrainSubsystem;
	private static ShooterSubsystem m_shooterSubsystem;
	private static IntakeSubsystem m_intakeSubsystem;
	private static TowerSubsystem m_towerSubsystem;

	public Auton_ThreeBall(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem,
			TowerSubsystem towerSubsystem,
			ShooterSubsystem shooterSubsystem) {
		m_commandScheduler = CommandScheduler.getInstance();

		m_drivetrainSubsystem = drivetrainSubsystem;
		m_intakeSubsystem = intakeSubsystem;
		m_towerSubsystem = towerSubsystem;
		m_shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void initialize() {

		NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable FMSInfo = inst.getTable("FMSInfo");
        NetworkTableEntry alienceColor = FMSInfo.getEntry("IsRedAlliance");
        boolean m_isRed = alienceColor.getBoolean(true);
        
		m_commandScheduler.schedule(new SequentialCommandGroup(
				new InstantCommand(() -> {
					m_drivetrainSubsystem.setBrakeMode(true);
				}),
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem, m_isRed ? Paths.redPaths.Start_Ball2 : Paths.bluePaths.Start_Ball2),
						new IntakeBalls(m_intakeSubsystem, m_towerSubsystem)),
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem, m_isRed ? Paths.redPaths.Ball2_Hub : Paths.bluePaths.Ball2_Hub),
						new IntakeBalls(m_intakeSubsystem, m_towerSubsystem).withTimeout(1)
								.andThen(new RunAutoTower(m_towerSubsystem))),
				new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 2),
				new ParallelDeadlineGroup(
						new FollowPath(m_drivetrainSubsystem, m_isRed ? Paths.redPaths.Hub_Ball3 : Paths.bluePaths.Hub_Ball3),
						new IntakeBalls(m_intakeSubsystem, m_towerSubsystem)),
				new ParallelCommandGroup(
						new FollowPath(m_drivetrainSubsystem, m_isRed ? Paths.redPaths.Ball3_Hub : Paths.bluePaths.Ball3_Hub),
						new IntakeBalls(m_intakeSubsystem, m_towerSubsystem).withTimeout(1)
								.andThen(new RunAutoTower(m_towerSubsystem))),
				new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 1),
				new InstantCommand(() -> {
					m_shooterSubsystem.setSpeed(0);
				}, m_shooterSubsystem)));
    }

	@Override
	public boolean isFinished() {
		return false;
	}
}
