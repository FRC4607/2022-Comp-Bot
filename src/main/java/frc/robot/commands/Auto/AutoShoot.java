package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class AutoShoot extends CommandBase {
    private final TowerSubsystem m_towerSubsystem;
    private final FlywheelSubsystem m_flywheelSubsystem;
    private Timer timer;

    private AUTOSHOOT_STATE state;

    private enum AUTOSHOOT_STATE {
        ACQUIRE_BALL,
        WAIT_FOR_FLYWHEEL,
        SHOOT,
        DONE
    }

    public AutoShoot(TowerSubsystem towerSubsystem, FlywheelSubsystem flywheelSubsystem) {
        m_towerSubsystem = towerSubsystem;
        m_flywheelSubsystem = flywheelSubsystem;
        addRequirements(towerSubsystem, flywheelSubsystem);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        checkForBalls();
    }

    @Override
    public void execute() {
        switch (state) {
            case ACQUIRE_BALL:
                if (!m_towerSubsystem.getHighBrakeBeam()) {
                    state = AUTOSHOOT_STATE.WAIT_FOR_FLYWHEEL;
                    timer.reset();
                    timer.start();
                    break;
                }
            case WAIT_FOR_FLYWHEEL:
                m_flywheelSubsystem.setRPM(FlywheelConstants.flywheeelRPM);
                if (m_flywheelSubsystem.constantSpeed() & timer.hasElapsed(0.1)) {
                    state = AUTOSHOOT_STATE.SHOOT;
                    break;
                }
            case SHOOT:
                if (m_towerSubsystem.getHighBrakeBeam()) {
                    checkForBalls();
                    break;
                }
                m_flywheelSubsystem.setTransferWheel(FlywheelConstants.transferWheelSpeed);
            case DONE:
                break;
        }
        if (m_towerSubsystem.getHighBrakeBeam() || m_towerSubsystem.getMidBrakeBeam()) {
            m_towerSubsystem.setSpeed(TowerConstants.agitatiorSpeed);
        }
    }

    private void checkForBalls() {
        // True if there is a ball in that spot.
        boolean high = !m_towerSubsystem.getHighBrakeBeam();
        boolean low = !m_towerSubsystem.getMidBrakeBeam();

        if (high) {
            state = AUTOSHOOT_STATE.WAIT_FOR_FLYWHEEL;
        } else if (low) {
            state = AUTOSHOOT_STATE.ACQUIRE_BALL;
        } else {
            state = AUTOSHOOT_STATE.DONE;
        }
    }

    @Override
    public boolean isFinished() {
        return state == AUTOSHOOT_STATE.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheelSubsystem.setTransferWheel(0);
        m_towerSubsystem.setSpeed(0);
        m_flywheelSubsystem.setSpeed(0);
    }
}
