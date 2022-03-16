package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TowerSubsystem.Color;

public class DriverIntake extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;
    private final TowerSubsystem m_towerSubsystem;
    private final XboxController m_driver;

    public DriverIntake(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem, XboxController driver) {
        m_intakeSubsystem = intakeSubsystem;
        m_towerSubsystem = towerSubsystem;
        addRequirements(m_intakeSubsystem);
        m_driver = driver;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Enabled Auto Tower", true);
    }

    @Override
    public void execute() {
        double speed = m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis() * IntakeConstants.reverseScalar;

        if (!SmartDashboard.getBoolean("Enabled Auto Tower", false)) {
            m_intakeSubsystem.setSpeed(speed * IntakeConstants.intakeSpeed);
        } else {
            if (speed > 0) {
                if (!m_towerSubsystem.getMidBrakeBeam() && !m_towerSubsystem.getHighBrakeBeam()) {
                    m_intakeSubsystem.setSpeed(0);
                    // m_intakeSubsystem.retractIntake();
                } else if ((m_towerSubsystem.getColorSensor() == m_towerSubsystem.getAllianceColor()
                        || m_towerSubsystem.getColorSensor() == Color.None
                        || m_towerSubsystem.getAllianceColor() == Color.None)) {
                    m_intakeSubsystem.setSpeed(speed * IntakeConstants.intakeSpeed);
                } else {
                    m_intakeSubsystem.setSpeed(0);
                }
            } else {
                m_intakeSubsystem.setSpeed(speed * IntakeConstants.intakeSpeed);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
