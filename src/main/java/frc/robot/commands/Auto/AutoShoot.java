package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.TransferWheelConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;

public class AutoShoot extends CommandBase {
    private final TowerSubsystem m_towerSubsystem;
    private final FlywheelSubsystem m_flywheelSubsystem;
    private final TransferWheelSubsystem m_transferWheelSubsystem;

    private AUTOSHOOT_STATE state;

    private enum AUTOSHOOT_STATE {
        ACQUIRE_BALL,
        WAIT_FOR_FLYWHEEL,
        SHOOT,
        DONE
    }
    
    public AutoShoot(TowerSubsystem towerSubsystem, FlywheelSubsystem flywheelSubsystem, TransferWheelSubsystem transferWheelSubsystem) {
        m_towerSubsystem = towerSubsystem;
        m_flywheelSubsystem = flywheelSubsystem;
        m_transferWheelSubsystem = transferWheelSubsystem;
        addRequirements(towerSubsystem, flywheelSubsystem, transferWheelSubsystem);
    }

    @Override
    public void initialize() {
        checkForBalls();
    }

    @Override
    public void execute() {
        switch (state) {
            case ACQUIRE_BALL:
                if (m_towerSubsystem.getHighBeam()) {
                    state = AUTOSHOOT_STATE.WAIT_FOR_FLYWHEEL;
                    break;
                }
                m_towerSubsystem.setSpeed(TowerConstants.agitatiorSpeed);
            case WAIT_FOR_FLYWHEEL:
                if (m_flywheelSubsystem.constantSpeed()) {
                    state = AUTOSHOOT_STATE.SHOOT;
                    break;
                }
                m_flywheelSubsystem.setRPM(FlywheelConstants.flywheeelRPM);
            case SHOOT:
                if (m_towerSubsystem.getHighBeam()) {
                    checkForBalls();
                    break;
                }
                m_transferWheelSubsystem.setTransferWheel(TransferWheelConstants.transferWheelSpeed);
            case DONE:
                break;
        }
    }

    private void checkForBalls() {
        // True if there is a ball in that spot.
        boolean high = !m_towerSubsystem.getHighBeam();
        boolean low = !m_towerSubsystem.getMidBeam();

        if (high) {
            state = AUTOSHOOT_STATE.WAIT_FOR_FLYWHEEL;
        }
        else if (low) {
            state = AUTOSHOOT_STATE.ACQUIRE_BALL;
        }
        else {
            state = AUTOSHOOT_STATE.DONE;
        }
    }

    @Override
    public boolean isFinished() {
        return state == AUTOSHOOT_STATE.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        m_transferWheelSubsystem.setTransferWheel(0);
        m_towerSubsystem.setSpeed(0);
        m_flywheelSubsystem.setSpeed(0);
    }
}
