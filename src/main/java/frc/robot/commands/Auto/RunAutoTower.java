package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.TowerSubsystem;

public class RunAutoTower extends CommandBase {
    private final TowerSubsystem m_towerSubsystem;

    public RunAutoTower(TowerSubsystem towerSubsystem) {
        m_towerSubsystem = towerSubsystem;

        addRequirements(m_towerSubsystem);
    }

    @Override
    public void execute() {
        if ((m_towerSubsystem.getMidBrakeBeam() && m_towerSubsystem.getHighBrakeBeam())) {
            m_towerSubsystem.setSpeed(0);
        } else {
            m_towerSubsystem.setSpeed(TowerConstants.agitatiorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_towerSubsystem.setSpeed(0);
    }
}