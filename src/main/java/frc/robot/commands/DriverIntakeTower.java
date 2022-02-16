package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TowerSubsystem.Color;

public class DriverIntakeTower extends CommandBase {

    private IntakeSubsystem m_intakeSubsystem;
    private TowerSubsystem m_towerSubsystem;
    private XboxController m_driver;

    public DriverIntakeTower(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem, XboxController driver) {
        m_intakeSubsystem = intakeSubsystem;
        m_towerSubsystem = towerSubsystem;
        addRequirements(m_intakeSubsystem, towerSubsystem);
        m_driver = driver;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis() * IntakeConstants.reverseScalar;
        m_intakeSubsystem.setSpeed(speed * IntakeConstants.intakeSpeed);

        if (m_towerSubsystem.getMidBrakeBeam() && m_towerSubsystem.getHighBrakeBeam()) {
            m_towerSubsystem.setSpeed(0);
            m_intakeSubsystem.retractIntake();
        } else if ((m_towerSubsystem.getColorSensor() == m_towerSubsystem.getAllianceColor()
                || m_towerSubsystem.getColorSensor() == Color.None
                || m_towerSubsystem.getAllianceColor() == Color.None)) {
            m_towerSubsystem.setSpeed(TowerConstants.agitatiorSpeed);
        } else {
            m_towerSubsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
