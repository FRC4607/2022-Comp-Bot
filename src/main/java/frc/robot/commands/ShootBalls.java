package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.*;

public class ShootBalls extends CommandBase {
	private final ShooterSubsystem m_shooterSubsystem;
	private final TowerSubsystem m_towerSubsystem;
	private final IntakeSubsystem m_intakeSubsystem;
	private Timer timer;

	enum sequence {
		spinupFlywheel,
		waitForBall,
		shoot
	}

	private sequence state;
	private boolean timerHasStarted = false;
	private int m_shots;
	private int m_takenShots;

	public ShootBalls(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
			int shots) {
		m_towerSubsystem = towerSubsystem;
		m_shooterSubsystem = shooterSubsystem;
		m_intakeSubsystem = intakeSubsystem;

		addRequirements(m_shooterSubsystem, m_towerSubsystem);

		m_shots = shots;
		m_takenShots = 0;
	}

	@Override
	public void initialize() {
		m_shooterSubsystem.spinupFlywheel();
		timer = new Timer();

		state = sequence.spinupFlywheel;
		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		switch (state) {
			case spinupFlywheel:
				if (m_shooterSubsystem.constantSpeed() && timer.hasElapsed(0.2)) {
					System.out.println("Spin up Time:" + timer.get());
					timer.stop();
					timer.reset();
					
					if (SmartDashboard.getBoolean("Enabled Auto Tower", false)) {
						state = sequence.waitForBall;
					} else {
						state = sequence.shoot;
					}
				}
				break;
			case waitForBall:

				if (!m_towerSubsystem.getHighBrakeBeam()) {
					if (timerHasStarted) {
						if (timer.hasElapsed(0.1)) {
							state = sequence.shoot;
							
							timer.stop();
							timer.reset();
							timer.start();

							m_shooterSubsystem.setKickerWheel(ShooterConstants.kickerWheelSpeed);
							timerHasStarted = false;
						}
					} else {
						timer.stop();
						timer.reset();
						timer.start();
						timerHasStarted = true;
					}
				} else {
				}
				
				break;
			case shoot:
				if (timer.hasElapsed(0.2)) {
					System.out.println("Recovering");
					m_shooterSubsystem.setKickerWheel(0);
					timer.reset();
					timer.start();
					state = sequence.spinupFlywheel;
					m_takenShots++;
				}
				break;
		}
		if (m_towerSubsystem.getHighBrakeBeam()) {
			m_towerSubsystem.setSpeed(TowerConstants.agitatiorSpeed);
		} else {
			m_towerSubsystem.setSpeed(0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_shooterSubsystem.setSpeed(0);
		m_shooterSubsystem.setKickerWheel(0);
		m_towerSubsystem.setSpeed(0);
	}

	@Override
	public boolean isFinished() {
		if (SmartDashboard.getBoolean("Enabled Auto Tower", false)) {
			return m_shots != 0 && m_takenShots >= m_shots;
		} else {
			return m_takenShots > 0;
		}
	}
}
