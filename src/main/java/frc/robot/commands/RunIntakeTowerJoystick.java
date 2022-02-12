package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class RunIntakeTowerJoystick extends CommandBase {
    
    private IntakeSubsystem m_intakeSubsystem;
    private TowerSubsystem m_towerSubsystem;
    private XboxController m_driver;

    public RunIntakeTowerJoystick(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem, XboxController driver) {
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
        m_towerSubsystem.setSpeed(speed * TowerConstants.agitatiorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
        m_intakeSubsystem.setSpeed(0);
    }
}
