package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TowerSubsystem.Color;

public class RunAutoTower extends CommandBase {
    private TowerSubsystem m_towerSubsystem;

    public RunAutoTower(TowerSubsystem towerSubsystem) {
        m_towerSubsystem = towerSubsystem;
    }

    @Override
    public void execute() {
        if ((m_towerSubsystem.getMidBrakeBeam() || m_towerSubsystem.getHighBrakeBeam()) &&
                (m_towerSubsystem.getColorSensor() == m_towerSubsystem.getAllianceColor()
                        || m_towerSubsystem.getColorSensor() == Color.None
                        || m_towerSubsystem.getAllianceColor() == Color.None)) {
            m_towerSubsystem.setSpeed(TowerConstants.agitatiorSpeed);
        } else {
            m_towerSubsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_towerSubsystem.setSpeed(0);
    }
}