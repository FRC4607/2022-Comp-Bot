package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class ShootBalls extends CommandBase {
	private FlywheelSubsystem m_flywheelSubsystem;
	private TowerSubsystem m_towerSubsystem;
	private Timer timer;

	enum sequence {
		spinupFlywheel,
		waitForBall,
		shoot
	}

	private sequence state;
	private boolean timerHasStarted;

	public ShootBalls(TowerSubsystem towerSubsystem, FlywheelSubsystem flywheelSubsystem) {
		m_towerSubsystem = towerSubsystem;
		m_flywheelSubsystem = flywheelSubsystem;

		addRequirements(m_flywheelSubsystem, m_towerSubsystem);
	}

	@Override
	public void initialize() {
		m_flywheelSubsystem.setRPM(SmartDashboard.getNumber("Flywheel Speed", FlywheelConstants.flywheeelRPM));
		timer = new Timer();

		state = sequence.shoot;
		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		switch (state) {
			case spinupFlywheel:
				if (m_flywheelSubsystem.constantSpeed() && timer.hasElapsed(0.1)) {
					System.out.println("Spin up Time:" + timer.get());
					
					if (!m_towerSubsystem.getHighBrakeBeam()) {
						timer.reset();
						timer.start();

						m_flywheelSubsystem.setTransferWheel(FlywheelConstants.transferWheelSpeed);
						state = sequence.shoot;
					} else {
						timer.stop();
						timer.reset();
						state = sequence.waitForBall;
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

							m_flywheelSubsystem.setTransferWheel(FlywheelConstants.transferWheelSpeed);
							timerHasStarted = false;
						}
					} else {
						timer.reset();
						timer.start();
						timerHasStarted = true;
					}
				}
				break;
			case shoot:
				if (timer.hasElapsed(0.2)) {
					System.out.println("Recovering");
					m_flywheelSubsystem.setTransferWheel(0);
					timer.reset();
					timer.start();
					state = sequence.spinupFlywheel;
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
		m_flywheelSubsystem.setSpeed(0);
		m_flywheelSubsystem.setTransferWheel(0);
	}
}
