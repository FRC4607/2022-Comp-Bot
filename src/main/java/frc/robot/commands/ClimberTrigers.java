package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTrigers extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    private final XboxController m_operator;

    public ClimberTrigers(ClimberSubsystem climberSubsystem, XboxController operator) {
        m_climberSubsystem = climberSubsystem;
        m_operator = operator;

        addRequirements(m_climberSubsystem);
    }

    @Override
    public void execute() {
        double speed = m_operator.getLeftTriggerAxis() * 1 - m_operator.getRightTriggerAxis() * 1;
        if (speed > 0.01 || speed < 0.01) {
            m_climberSubsystem.setClimber(speed);
        }
    }
}
