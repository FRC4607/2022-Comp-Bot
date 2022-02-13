package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TowerSubsystem;

public class AutoTower extends CommandBase {
    private TowerSubsystem m_towerSubsystem;

    public AutoTower(TowerSubsystem towerSubsystem) {
        m_towerSubsystem = towerSubsystem;
        addRequirements(m_towerSubsystem);
    
    }
    @Override
    public void initialize() {
        m_towerSubsystem.AutoTower(true);
    }

    @Override
    public void end(boolean interrupted) {
        m_towerSubsystem.AutoTower(false);
    }


}
