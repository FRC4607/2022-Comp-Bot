package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.TowerSubsystem;

public class RunTower extends CommandBase {
    private final boolean m_reverse;
    private final TowerSubsystem m_towerSubsystem;

    public RunTower(TowerSubsystem towerSubsystem, boolean reverse) {
        m_towerSubsystem = towerSubsystem;
        m_reverse = reverse;
        addRequirements(m_towerSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!m_reverse) {
            m_towerSubsystem.setSpeed(TowerConstants.agitatiorSpeed);
        }
        else {
            m_towerSubsystem.setSpeed(-TowerConstants.agitatiorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_towerSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

