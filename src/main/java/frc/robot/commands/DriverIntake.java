package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TowerSubsystem.Color;

public class DriverIntake extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;
    private final TowerSubsystem m_towerSubsystem;
    private final XboxController m_driver;

    public DriverIntake(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem,
            XboxController driver) {
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
        double speed = m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis();

        if ((speed > 0.1 && !(m_towerSubsystem.getMidBrakeBeam() && m_towerSubsystem.getHighBrakeBeam()))) {
            m_intakeSubsystem.setSpeed(Constants.IntakeConstants.intakeSpeed * speed);
        } else if (speed < -0.1) {
            m_intakeSubsystem.setSpeed(Constants.IntakeConstants.intakeSpeed * speed);
        } else {
            m_intakeSubsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
