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
    private final ShooterSubsystem m_shooterSubsystem;
    private final XboxController m_driver;

    public DriverIntake(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem, ShooterSubsystem flywheel, XboxController driver) {
        m_intakeSubsystem = intakeSubsystem;
        m_towerSubsystem = towerSubsystem;
        m_shooterSubsystem = flywheel;
        addRequirements(m_intakeSubsystem);
        m_driver = driver;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Enabled Auto Tower", true);
    }

    @Override
    public void execute() {
        double speed = m_driver.getLeftTriggerAxis();

        if (speed > 0.1) {
            m_intakeSubsystem.setSpeed(-Constants.IntakeConstants.intakeSpeed * speed);
            m_towerSubsystem.setSpeed(-Constants.TowerConstants.agitatiorSpeed * speed);
            m_shooterSubsystem.setKickerWheel(-Constants.ShooterConstants.kickerWheelSpeed * speed);
            m_shooterSubsystem.setSpeed(-0.5 * speed);
        }
        else {
            m_intakeSubsystem.setSpeed(0);
            m_towerSubsystem.setSpeed(0);
            m_shooterSubsystem.setKickerWheel(0);
            m_shooterSubsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
        m_towerSubsystem.setSpeed(0);
        m_shooterSubsystem.setKickerWheel(0);
        m_shooterSubsystem.setSpeed(0);
    }
}
