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

	private int cycleCount = 0;

	private final int m_targetRPM;

	public ShootBalls(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
			int shots, int targetRPM) {
		m_towerSubsystem = towerSubsystem;
		m_shooterSubsystem = shooterSubsystem;

		addRequirements(m_shooterSubsystem, m_towerSubsystem);

		m_shots = shots;

		m_targetRPM = targetRPM;
	}

	public ShootBalls(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, int shots) {
		this(towerSubsystem, shooterSubsystem, intakeSubsystem, shots, 0);
	}

	@Override
	public void initialize() {
		if (m_targetRPM != 0) {
			m_shooterSubsystem.spinupFlywheel(m_targetRPM);
		}
		else {
			m_shooterSubsystem.spinupFlywheel();
		}
		timer = new Timer();
		m_takenShots = 0;
		state = sequence.spinupFlywheel;
		timer.reset();
		timer.start();
		cycleCount = 0;
	}

	@Override
	public void execute() {
		cycleCount++;
		switch (state) {
			case spinupFlywheel:
				if (m_shooterSubsystem.constantSpeed() && cycleCount >= 5) {
					System.out.println("Spin up Time:" + timer.get());

					cycleCount = 0;
					state = sequence.waitForBall;
				}
				break;
			case waitForBall:

				if (m_towerSubsystem.getHighBrakeBeam()) {
					if (timerHasStarted) {
						if (cycleCount >= 5) {
							state = sequence.shoot;

							cycleCount = 0;

							m_shooterSubsystem.setKickerWheel(ShooterConstants.kickerWheelSpeed);
							timerHasStarted = false;
						}
					} else {

						cycleCount = 0;

						timerHasStarted = true;
					}
				} else {
				}

				break;
			case shoot:
				if (m_takenShots == m_shots - 1) {
					if (cycleCount >= 10) {
						m_shooterSubsystem.setKickerWheel(0);
	
						cycleCount = 0;
	
						timer.reset();
						timer.start();
						state = sequence.spinupFlywheel;
						m_takenShots++;
					}
				}
				else {
					if (cycleCount >= 5) {
						m_shooterSubsystem.setKickerWheel(0);
	
						cycleCount = 0;
	
						timer.reset();
						timer.start();
						state = sequence.spinupFlywheel;
						m_takenShots++;
					}
				}
				
				break;
		}
		if (m_towerSubsystem.getHighBrakeBeam()) {
			m_towerSubsystem.setSpeed(0);
		} else {
			m_towerSubsystem.setSpeed(TowerConstants.agitatiorSpeed);
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
		return m_shots != 0 && m_takenShots >= m_shots;
	}
}
