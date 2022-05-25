package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.TowerSubsystem;

public class DriverTower extends CommandBase {

    private final TowerSubsystem m_towerSubsystem;
    private final XboxController m_driver;

    public DriverTower(TowerSubsystem towerSubsystem, XboxController driver) {

        m_towerSubsystem = towerSubsystem;
        addRequirements(towerSubsystem);
        m_driver = driver;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis();

        if (!SmartDashboard.getBoolean("Enabled Auto Tower", false)) {
            m_towerSubsystem.setSpeed(speed * TowerConstants.agitatiorSpeed);
        } else {
            if (speed > 0) {
                if (m_towerSubsystem.getMidBrakeBeam() && m_towerSubsystem.getHighBrakeBeam()) {
                    m_towerSubsystem.setSpeed(0);
                } else {
                    m_towerSubsystem.setSpeed(speed * TowerConstants.agitatiorSpeed);
                }
            } else {
                m_towerSubsystem.setSpeed(speed * TowerConstants.agitatiorSpeed);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_towerSubsystem.setSpeed(0);
    }
}